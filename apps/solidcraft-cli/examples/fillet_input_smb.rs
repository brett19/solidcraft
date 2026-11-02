//! Saves the body a kernel case's last `solid.fillet` is given as a native SMLib `.smb`, with the
//! edge SolidCraft's pick resolves to, for timing SMLib's fillet on its own (perf-inputs/).
//! Usage: fillet_input_smb CASE-DIR OUT.smb
//! Runs the case's recipe without its last command (which must be `solid.fillet`), saves the
//! named body (or the only one) as SMLib holds it, reads the file back and runs the same round on
//! it as `crates/kernel/src/blend_smlib.rs` does: the edge's midpoint, at the pick tolerance.
use serde_json::{Value, json};
use solidcraft_engine::{Session, geom::Vec3, kernel};

fn word<T: std::fmt::Debug>(v: &T) -> String {
    let s = format!("{v:?}");
    s.split(|c: char| !c.is_alphanumeric()).next().unwrap_or("").to_string()
}

fn polyline_mid(pts: &[Vec3]) -> Vec3 {
    let len: f64 = pts.windows(2).map(|w| w.first().zip(w.get(1)).map(|(a, b)| a.dist(*b)).unwrap_or(0.0)).sum();
    let mut acc = 0.0;
    for w in pts.windows(2) {
        if let (Some(a), Some(b)) = (w.first(), w.get(1)) {
            let l = a.dist(*b);
            if acc + l >= len * 0.5 && l > 0.0 {
                return a.lerp(*b, (len * 0.5 - acc) / l);
            }
            acc += l;
        }
    }
    pts.first().copied().unwrap_or_default()
}

fn run() -> Result<Value, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [case, out] = args.as_slice() else {
        return Err("usage: fillet_input_smb CASE-DIR OUT.smb".into());
    };
    let text = std::fs::read_to_string(format!("{case}/recipe.json")).map_err(|e| e.to_string())?;
    let mut recipe: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let commands = recipe["commands"].as_array_mut().ok_or("recipe has no commands")?;
    let fillet = commands.pop().ok_or("empty recipe")?;
    if fillet["command"] != "solid.fillet" {
        return Err(format!("last command is {}, not solid.fillet", fillet["command"]));
    }
    let radius = fillet["params"]["radius"].as_f64().ok_or("fillet radius")?;
    let picks: Vec<Vec3> = fillet["params"]["edges"]
        .as_array()
        .ok_or("fillet edges")?
        .iter()
        .map(|p| Vec3::new(p[0].as_f64().unwrap_or(f64::NAN), p[1].as_f64().unwrap_or(f64::NAN), p[2].as_f64().unwrap_or(f64::NAN)))
        .collect();
    let mut s = Session::default();
    s.run_script(&recipe).map_err(|e| e.to_string())?;
    let world = s.world_state().map_err(|e| e.to_string())?;
    let name = fillet["params"]["body"].as_str();
    let mb = match name {
        Some(n) => world.bodies.iter().find(|b| b.name == n),
        None if world.bodies.len() == 1 => world.bodies.first(),
        None => None,
    }
    .ok_or("the fillet's body is not in the model")?;
    let body = &mb.body;

    // The edge as the SMLib blend resolves it (blend_smlib::resolve).
    let edges = kernel::blend_edges(body, &picks).map_err(|e| e.to_string())?;
    let mesh = body.mesh(&kernel::MeshTolerance::chord((body.size() * 2e-3).max(1e-3))).map_err(|e| e.to_string())?;
    let tol = body.size() * 0.05 + 1e-3;
    let record = body.record();
    let mut described = Vec::new();
    let mut mids = Vec::new();
    for &i in &edges {
        let e = record.edges.get(i).ok_or("picked edge out of range")?;
        let mid = polyline_mid(mesh.edges.get(i).map(Vec::as_slice).unwrap_or(&[]));
        mids.push([mid.x, mid.y, mid.z]);
        let faces: Vec<Value> = e
            .uses
            .iter()
            .map(|u| {
                let f = record.faces.get(u.face as usize);
                json!({"face": u.face, "surface": f.map(|f| word(&f.surface)).unwrap_or_default()})
            })
            .collect();
        let ends: Vec<[f64; 3]> = e.vertices.iter().filter_map(|v| record.vertices.get(*v as usize).map(|v| v.point)).collect();
        described.push(json!({
            "edge": i,
            "curve": word(&e.curve),
            "closed": e.vertices[0] == e.vertices[1],
            "endpoints": ends,
            "faces": faces,
            "smlib_pick_point": [mid.x, mid.y, mid.z],
        }));
    }

    let sm = kernel::smlib_body(body).map_err(|e| e.to_string())?;
    let path = std::path::Path::new(out);
    sm.write(path, false).map_err(|e| e.to_string())?;
    let back = solidcraft_smlib::Brep::read(path, false).map_err(|e| format!("read back: {e}"))?;
    let counts = back.counts().map_err(|e| e.to_string())?;
    let volume_in = back.volume(1e-7).map_err(|e| e.to_string())?;
    let radii = vec![radius; mids.len()];
    let clock = std::time::Instant::now();
    let rounded = back.blend_dropping(solidcraft_smlib::Blend::Round, &mids, &radii, tol);
    let seconds = clock.elapsed().as_secs_f64();
    let result = match rounded {
        Ok((r, dropped)) => json!({
            "ok": true,
            "seconds": seconds,
            "volume": r.volume(1e-7).map_err(|e| e.to_string())?,
            "counts": r.counts().map_err(|e| e.to_string())?,
            "valid": r.check(solidcraft_smlib::CheckLevel::All).map(|c| c.is_valid()).map_err(|e| e.to_string())?,
            "dropped": dropped,
        }),
        Err(e) => json!({"ok": false, "seconds": seconds, "error": e.to_string()}),
    };
    Ok(json!({
        "case": case,
        "body": mb.name,
        "smb": out,
        "bytes": std::fs::metadata(path).map(|m| m.len()).unwrap_or(0),
        "read_back": {"faces_edges_vertices": counts, "volume": volume_in},
        "radius": radius,
        "picks": fillet["params"]["edges"],
        "pick_tolerance": tol,
        "edges": described,
        "fillet_on_read_back": result,
    }))
}

fn main() {
    match run() {
        Ok(v) => println!("{}", serde_json::to_string_pretty(&v).unwrap_or_default()),
        Err(e) => {
            eprintln!("fillet_input_smb: {e}");
            std::process::exit(1);
        }
    }
}
