use super::*;

#[cfg(feature = "native")]
#[test]
fn box_end_to_end() {
    let b = Brep::make_box([1.0, 2.0, 3.0], [10.0, 20.0, 30.0]).unwrap();
    assert!((b.volume(1e-7).unwrap() - 6000.0).abs() < 1e-6);
    assert_eq!(b.counts().unwrap(), [6, 12, 8]);
    let c = b.try_clone().unwrap();
    drop(b);
    assert!((c.volume(1e-7).unwrap() - 6000.0).abs() < 1e-6);
    assert_eq!(current_call(), None, "no call in flight between calls");
}

#[cfg(feature = "native")]
#[test]
fn bad_arguments_are_errors() {
    assert_eq!(Brep::make_box([0.0; 3], [1.0, -1.0, 1.0]).unwrap_err(), Error::Argument);
    assert_eq!(Brep::make_box([f64::NAN, 0.0, 0.0], [1.0; 3]).unwrap_err(), Error::Argument);
    // each kernel refuses a bad box the same way, whichever one `Op::Primitive` routes to
    init().unwrap();
    for size in [[1.0, -1.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, f64::NAN]] {
        assert_eq!(core::make_box([0.0; 3], size).err(), Some(Error::Argument), "solidcore: {size:?}");
        assert_eq!(ffi::make_box(&[0.0; 3], &size).map_err(error).err(), Some(Error::Argument), "C++: {size:?}");
    }
    let b = Brep::make_box([0.0; 3], [1.0; 3]).unwrap();
    assert_eq!(b.volume(f64::INFINITY).unwrap_err(), Error::Argument);
}

/// `.smb` files on solidcore, in both forms and with a curved body, and a missing or corrupt file
/// is an error, not a panic.
#[test]
fn solidcore_reads_back_the_files_it_writes() {
    use std::f64::consts::PI;
    let dir = std::env::temp_dir().join(format!("solidcraft-smlib-core-file-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let cylinder = core::make_cylinder([1.0, 2.0, 3.0], 2.0, 5.0).map(Brep::made_core).unwrap();
    for ascii in [false, true] {
        let path = dir.join(format!("cylinder-{ascii}.smb"));
        core::file::write(cylinder.core().unwrap(), &path, ascii).unwrap();
        let back = core::file::read(&path, ascii).map(Brep::made_core).unwrap();
        let v = core::volume(back.core().unwrap(), 1e-7).unwrap();
        assert!((v - PI * 4.0 * 5.0).abs() < 1e-5, "ascii {ascii}: {v}");
        assert_eq!(core::counts(back.core().unwrap()).unwrap(), core::counts(cylinder.core().unwrap()).unwrap(), "ascii {ascii}");
    }
    let body = cylinder.core().unwrap();
    assert!(core::file::write(body, &dir.join("missing/x.smb"), false).is_err());
    assert!(core::file::read(&dir.join("missing.smb"), false).is_err());
    let junk = dir.join("junk.smb");
    std::fs::write(&junk, b"not a B-rep file").unwrap();
    assert!(core::file::read(&junk, false).is_err());
    let _ = std::fs::remove_dir_all(&dir);
}

/// ASCII that is no B-rep file is refused like binary. It comes back as an empty body today:
/// default-solidcore/bugs/ascii-junk-reads-as-empty.md.
#[test]
#[ignore = "solidcore: an ASCII file of junk reads as an empty B-rep instead of an error"]
fn solidcore_refuses_an_ascii_file_of_junk() {
    let dir = std::env::temp_dir().join(format!("solidcraft-smlib-core-junk-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let junk = dir.join("junk.smb");
    std::fs::write(&junk, b"not a B-rep file").unwrap();
    let r = core::file::read(&junk, true);
    let _ = std::fs::remove_dir_all(&dir);
    assert!(r.is_err());
}

/// A file either kernel wrote reads on the other, with the same measure.
#[cfg(feature = "native")]
#[test]
fn each_kernel_reads_the_files_of_the_other() {
    use std::f64::consts::PI;
    let dir = std::env::temp_dir().join(format!("solidcraft-smlib-cross-file-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let want = PI * 4.0 * 5.0;
    for ascii in [false, true] {
        let from_core = dir.join(format!("core-{ascii}.smb"));
        let body = core::make_cylinder([0.0; 3], 2.0, 5.0).unwrap();
        core::file::write(&body, &from_core, ascii).unwrap();
        crate::init().unwrap();
        let on_cpp = ffi::read(&c_path(&from_core).unwrap(), ascii).map(Brep::made).unwrap();
        let v = ffi::volume(on_cpp.cpp().unwrap(), 1e-7).unwrap();
        assert!((v - want).abs() < 1e-5, "C++ on solidcore's file, ascii {ascii}: {v}");
        let from_cpp = dir.join(format!("cpp-{ascii}.smb"));
        let cpp = ffi::make_cylinder(&[0.0; 3], 2.0, 5.0).map(Brep::made).unwrap();
        crate::check(ffi::write(cpp.cpp().unwrap(), &c_path(&from_cpp).unwrap(), ascii)).unwrap();
        let on_core = core::file::read(&from_cpp, ascii).unwrap();
        let v = core::volume(&on_core, 1e-7).unwrap();
        assert!((v - want).abs() < 1e-5, "solidcore on C++'s file, ascii {ascii}: {v}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[cfg(feature = "native")]
#[test]
fn native_files_round_trip_and_failures_dump_their_inputs() {
    let _cpp = route::on_cpp();
    let dir = std::env::temp_dir().join(format!("solidcraft-smlib-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let b = Brep::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap();
    let path = dir.join("box.smb");
    b.write(&path, false).unwrap();
    let back = Brep::read(&path, false).unwrap();
    assert!((back.volume(1e-7).unwrap() - 24.0).abs() < 1e-9);

    let dumps = dir.join("dumps");
    std::fs::create_dir_all(&dumps).unwrap();
    set_dump_dir(Some(&dumps)).unwrap();
    assert!(b.write(&dir.join("missing/x.smb"), false).is_err());
    set_dump_dir(None).unwrap();
    let names: Vec<String> =
        std::fs::read_dir(&dumps).unwrap().filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    assert!(names.iter().any(|n| n.ends_with("-smlib_brep_write.txt")), "{names:?}");
    let smb = names.iter().find(|n| n.ends_with("-smlib_brep_write-brep.smb")).unwrap();
    let replayed = Brep::read(&dumps.join(smb), false).unwrap();
    assert!((replayed.volume(1e-7).unwrap() - 24.0).abs() < 1e-9);
    let _ = std::fs::remove_dir_all(&dir);
}

#[cfg(feature = "native")]
#[test]
fn calls_from_many_threads_are_serialised() {
    let handles: Vec<_> = (0..8)
        .map(|i| {
            std::thread::spawn(move || {
                let s = 1.0 + i as f64;
                let b = Brep::make_box([0.0; 3], [s; 3]).unwrap();
                (b.volume(1e-7).unwrap() - s * s * s).abs() < 1e-9 * s * s * s
            })
        })
        .collect();
    assert!(handles.into_iter().all(|h| h.join().unwrap()));
}

#[cfg(not(feature = "native"))]
#[test]
fn without_native_every_call_says_so() {
    assert_eq!(init(), Err(Error::NotBuilt));
    // the C API's own box, whatever `Op::Primitive` routes to
    assert_eq!(ffi::make_box(&[0.0; 3], &[1.0; 3]).map_err(error).err(), Some(Error::NotBuilt));
    // solidcore builds a box without SMLib; asking the C++ kernel about it says there is none
    let b = core::make_box([0.0; 3], [1.0; 3]).map(Brep::made_core).unwrap();
    assert_eq!(b.cpp().err(), Some(Error::NotBuilt));
    assert_eq!(current_call(), None);
}

#[test]
fn call_lines_give_the_function_and_the_c_api_time() {
    let (name, took) = call_time("smlib_boolean(op=1, a=#3 (12 faces), b=#4) -> 0 (12.345 ms)").unwrap();
    assert_eq!((name, took), ("smlib_boolean", std::time::Duration::from_micros(12_345)));
    assert_eq!(call_time("smlib_make_box(origin=(0,0,0)) -> -1002 (0.001 ms)").map(|c| c.0), Some("smlib_make_box"));
    for bad in ["SMLib warning at x.cpp:12: tolerance (in smlib_boolean(...))", "smlib_x() -> 0 (nan ms)", "smlib_x() -> 0 (-1 ms)", "", "smlib_x"] {
        assert_eq!(call_time(bad), None, "{bad}");
    }
}

#[cfg(feature = "native")]
#[test]
fn every_call_is_a_profiler_span_timed_by_the_c_api() {
    let _cpp = route::on_cpp();
    solidcraft_profile::set_enabled(true);
    let mark = solidcraft_profile::mark();
    // the C API's own box, whatever `Op::Primitive` routes to
    crate::init().unwrap();
    let b = ffi::make_box(&[0.0; 3], &[1.0; 3]).map(Brep::made).unwrap();
    let _ = b.volume(1e-7).unwrap();
    let r = mark.report();
    let names: Vec<&str> = r.rows.iter().filter(|r| r.category == "smlib").map(|r| r.name).collect();
    assert!(names.contains(&"smlib_make_box") && names.contains(&"smlib_brep_volume"), "{r:?}");
}

/// SMLib may end an edge at a parameter of its own (a knot its vertex lies at, where its drop onto
/// the patch ends): within the edge's tolerance as SMLib held it, on its curve, the pcurve is
/// mapped onto our interval exactly; farther is SMLib trimming the edge, refused with how far. An
/// edge whose gap from its surface raised its tolerance (two of Poppy Humanoid's left thigh, SMLib
/// ending them 0.1 um short) is held at that tolerance.
#[test]
fn a_pcurve_smlib_ends_within_the_edges_tolerance_is_mapped_onto_our_interval() {
    use solidcraft_geom::brep::pcurve::TrimEdge;
    use solidcraft_geom::brep::{Curve, Curve2};
    let line = Curve::Line { start: [0.0, 0.0, 0.0], end: [2.0, 0.0, 0.0] };
    let edge = TrimEdge { curve: &line, nurbs: None, interval: [0.0, 1.0], vertices: [0, 1], tolerance: 1e-5 };
    let theirs = [0.0, 1.0 - 2e-8];
    let pc = Curve2::Spline { knots: vec![0.0, 0.0, theirs[1], theirs[1]], points: vec![[0.0, 0.0], [1.0, 0.0]] };
    let mapped = crate::pcurves::on_our_interval(pc.clone(), &edge, theirs, edge.tolerance).unwrap();
    assert_eq!(mapped.interval(), Some([0.0, 1.0]));
    assert_eq!(mapped.eval(1.0), Some([1.0, 0.0]));
    // 0.02 mm along the edge is past its 1e-5 mm.
    let err = crate::pcurves::on_our_interval(pc, &edge, [0.0, 0.99], edge.tolerance).unwrap_err();
    assert!(err.contains("SMLib moved an edge's interval") && err.contains("tolerance"), "{err}");
    // Held at 0.03 mm, as an edge 0.01 mm off its surface is, the same end is within it.
    let short = Curve2::Spline { knots: vec![0.0, 0.0, 0.99, 0.99], points: vec![[0.0, 0.0], [1.0, 0.0]] };
    let held = crate::pcurves::on_our_interval(short, &edge, [0.0, 0.99], 0.03).unwrap();
    assert_eq!(held.interval(), Some([0.0, 1.0]));
    assert_eq!(held.eval(1.0), Some([1.0, 0.0]));
}

/// An edge whose curve runs 2.5 um past its vertex and its face's patch (as Poppy Humanoid's
/// Congé19 has them), with a tolerance of 0.5 um: SMLib trims such an edge back to the patch
/// when it makes its trim, unless the edge's tolerance covers its gap. The gap is measured
/// (SMLib's own, over the patch) and covered, so the face's pcurves come back on the edge's
/// own interval, ending at the patch's corner.
#[cfg(feature = "native")]
#[test]
fn an_edge_a_few_um_past_its_patch_keeps_its_interval() {
    use solidcraft_geom::brep::pcurve::{TrimEdge, TrimFace, TrimUse, Trims};
    use solidcraft_geom::brep::{Curve, Surface};
    // S(u, v) = (10u, 10v, 2v^2), bicubic
    let g = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0];
    let z = [0.0, 0.0, 2.0 / 3.0, 2.0];
    let bezier = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let points: Vec<Vec<[f64; 3]>> = (0..4).map(|i| (0..4).map(|j| [10.0 * g[i], 10.0 * g[j], z[j]]).collect()).collect();
    let surface = Surface::Spline { u_knots: bezier.clone(), v_knots: bezier.clone(), points: points.clone() };
    let past = 2.5e-3;
    let spline = |p: Vec<[f64; 3]>| Curve::Spline { knots: bezier.clone(), points: p };
    let curves = [
        spline((0..4).map(|i| [(10.0 + past) * g[i], 0.0, 0.0]).collect()),
        spline((0..4).map(|j| points[3][j]).collect()),
        spline((0..4).map(|i| points[3 - i][3]).collect()),
        spline((0..4).map(|j| points[0][3 - j]).collect()),
    ];
    let ends = [[0, 1], [1, 2], [2, 3], [3, 0]];
    let edges = (0..4)
        .map(|k| TrimEdge { curve: &curves[k], nurbs: None, interval: [0.0, 1.0], vertices: ends[k], tolerance: if k == 0 { 1e-3 } else { 1e-5 } })
        .collect();
    let face = TrimFace {
        surface: &surface,
        nurbs: None,
        domain: [[0.0, 1.0], [0.0, 1.0]],
        edges,
        vertices: vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 2.0], [0.0, 10.0, 2.0]],
        loops: vec![(0..4).map(|k| TrimUse { edge: k, forward: true, pcurve: None }).collect()],
        tolerance: 1e-5,
    };
    let pcs = crate::pcurves::SmlibTrims.trims(&face).unwrap();
    let pc = &pcs[0][0];
    assert_eq!(pc.interval(), Some([0.0, 1.0]));
    let [u, v] = pc.eval(1.0).unwrap();
    assert!((u - 1.0).abs() < 1e-9 && v.abs() < 1e-9, "{u} {v}");
}
