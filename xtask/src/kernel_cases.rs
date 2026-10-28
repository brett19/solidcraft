//! `cargo xtask kernel-cases`: the kernel regression rig. Runs every case in
//! `tests/kernel-cases/<area>/<id>/` through `solidcraft-cli kernel-cases` on SMLib, one process
//! per case (so a crash or a hang is that case's result, not the run's), and writes
//! `docs/kernel-cases.md`: results per case, coverage per area and per source, and the issues and
//! PRs looked at and skipped. Each case is graded against its own answer.
//!
//! The run's output folder (`--out`, default `target/kernel-cases`) holds `results.json`, every
//! case's full result, and `dumps/<area>-<case>/`, the inputs of each SMLib call a case's run
//! failed in (`solidcraft-cli kernel-cases --dumps`), for replay.
//!
//! `--kernel smlib|solidcore|both` picks the backend each case's process runs on
//! (`SOLIDCRAFT_KERNEL`). A solidcore run keeps its results, dumps and report in `<out>/solidcore/`;
//! `both` runs every case on each, keeps `docs/kernel-cases.md` as the SMLib report and writes the
//! side by side to `docs/kernel-cases-backends.md` and `<out>/backends.json`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use crate::{cargo, root};

const DEFAULT_TIMEOUT_S: u64 = 180;

/// The backends `--kernel` may name, as `SOLIDCRAFT_KERNEL` takes them.
const BACKENDS: [&str; 2] = ["smlib", "solidcore"];

/// How many cases ended in each status.
type StatusCounts = BTreeMap<String, usize>;

pub(crate) struct Options {
    cli: Option<String>,
    area: Option<String>,
    timeout: Duration,
    pub(crate) jobs: usize,
    /// The run's output folder: `results.json` and the SMLib dumps (`dumps/`).
    out: PathBuf,
    /// The backends each case runs on (`--kernel`, default SMLib alone).
    pub(crate) kernels: Vec<&'static str>,
}

impl Options {
    /// A backend's output folder: the run's own for SMLib, `<out>/<backend>/` for any other.
    fn out_of(&self, kernel: &str) -> PathBuf {
        if kernel == "smlib" { self.out.clone() } else { self.out.join(kernel) }
    }
}

/// The run's options; `out` is its output folder unless `--out` gives one.
pub(crate) fn options(rest: &[&str], out: &str) -> Result<Options, String> {
    let arg = |name: &str| rest.iter().position(|a| *a == name).and_then(|i| rest.get(i + 1)).map(|s| s.to_string());
    let secs = arg("--timeout").map(|t| t.parse::<u64>().map_err(|e| format!("--timeout: {e}"))).transpose()?.unwrap_or(DEFAULT_TIMEOUT_S);
    let jobs = arg("--jobs").map(|j| j.parse::<usize>().map_err(|e| format!("--jobs: {e}"))).transpose()?.unwrap_or(4).clamp(1, 32);
    let out = arg("--out").map_or_else(|| root().join(out), PathBuf::from);
    let kernels = match arg("--kernel").as_deref() {
        None | Some("smlib") => vec![BACKENDS[0]],
        Some("solidcore") => vec![BACKENDS[1]],
        Some("both") => BACKENDS.to_vec(),
        Some(k) => return Err(format!("--kernel: unknown backend '{k}' (smlib, solidcore or both)")),
    };
    Ok(Options { cli: arg("--cli"), area: arg("--area"), timeout: Duration::from_secs(secs), jobs, out, kernels })
}

/// Every case's full result as JSON, into the run's output folder.
pub(crate) fn save_results(o: &Options, results: &[Value]) -> Result<(), String> {
    save_json(&o.out, "results.json", results)
}

/// `value` as pretty JSON into `dir/name`.
fn save_json<T: serde::Serialize + ?Sized>(dir: &Path, name: &str, value: &T) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let path = dir.join(name);
    let text = serde_json::to_string_pretty(value).map_err(|e| e.to_string())?;
    std::fs::write(&path, text).map_err(|e| format!("{}: {e}", path.display()))
}

pub(crate) fn read(path: &Path) -> Value {
    std::fs::read_to_string(path).ok().and_then(|t| serde_json::from_str(&t).ok()).unwrap_or(Value::Null)
}

/// Every case directory, sorted by area then id.
fn cases(dir: &Path, only: Option<&str>) -> Result<Vec<PathBuf>, String> {
    let mut out = Vec::new();
    for area in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?.flatten() {
        let area = area.path();
        if !area.is_dir() || only.is_some_and(|a| area.file_name().is_none_or(|n| n != a)) {
            continue;
        }
        for case in std::fs::read_dir(&area).map_err(|e| format!("{}: {e}", area.display()))?.flatten() {
            let case = case.path();
            if case.join("recipe.json").is_file() && case.join("expected.json").is_file() {
                out.push(case);
            }
        }
    }
    out.sort();
    Ok(out)
}

/// One case, in its own process, killed after the timeout; `path` is the case's directory under
/// `tests/kernel-cases/`.
fn run_one(o: &Options, case: &Path, kernel: &str) -> Value {
    let mut r = run_case(o, case, kernel);
    let base = root().join("tests/kernel-cases");
    r["path"] = json!(case.strip_prefix(&base).unwrap_or(case).to_string_lossy().replace('\\', "/"));
    r
}

fn run_case(o: &Options, case: &Path, kernel: &str) -> Value {
    let rel = case.strip_prefix(root()).unwrap_or(case).to_path_buf();
    let mut cmd = match &o.cli {
        Some(bin) => Command::new(bin),
        None => {
            let mut c = cargo();
            c.args(["run", "-q", "--release", "-p", "solidcraft-cli", "--features", "smlib", "--"]);
            c
        }
    };
    let mut dumps = std::ffi::OsString::from("--dumps=");
    dumps.push(o.out_of(kernel).join("dumps"));
    cmd.env("SOLIDCRAFT_KERNEL", kernel)
        .current_dir(root())
        .args(["kernel-cases", "--json"])
        .arg(dumps)
        .arg(&rel)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let base = |status: &str, error: String| {
        let area = case.parent().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let id = case.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        json!({"case": id, "area": area, "status": status, "error": error})
    };
    let mut child = match cmd.spawn() {
        Ok(c) => c,
        Err(e) => return base("fail", format!("solidcraft-cli: {e}")),
    };
    // A case may need longer than the run's timeout (`timeout_s`, at most 30 min).
    let timeout = read(&case.join("expected.json"))["timeout_s"].as_u64().map_or(o.timeout, |s| Duration::from_secs(s.min(1800)).max(o.timeout));
    let start = Instant::now();
    loop {
        match child.try_wait() {
            Ok(Some(_)) => break,
            Ok(None) if start.elapsed() > timeout => {
                let _ = child.kill();
                let _ = child.wait();
                return base("hang", format!("no result after {} s", timeout.as_secs()));
            }
            Ok(None) => std::thread::sleep(Duration::from_millis(50)),
            Err(e) => return base("fail", format!("wait: {e}")),
        }
    }
    let out = match child.wait_with_output() {
        Ok(o) => o,
        Err(e) => return base("fail", format!("output: {e}")),
    };
    match serde_json::from_slice::<Value>(&out.stdout).ok().and_then(|v| v.as_array().and_then(|a| a.first().cloned())) {
        Some(mut r) => {
            // `seconds` is the commands' own time, measured inside the case's process.
            r["process_seconds"] = json!(start.elapsed().as_secs_f64());
            r
        }
        // No JSON: the process died (a native crash, an abort).
        None => {
            let err = String::from_utf8_lossy(&out.stderr);
            let tail: String = err.lines().rev().take(3).collect::<Vec<_>>().into_iter().rev().collect::<Vec<_>>().join(" / ");
            base("crash", format!("process exited ({}) without a result: {tail}", out.status))
        }
    }
}

/// Every case on SMLib.
pub(crate) fn run_all(o: &Options, list: &[PathBuf]) -> Vec<Value> {
    run_on(o, list, &["smlib"]).into_iter().next().unwrap_or_default()
}

/// Every case on each backend, one job pool for all the runs; the results per backend, in order.
fn run_on(o: &Options, list: &[PathBuf], kernels: &[&str]) -> Vec<Vec<Value>> {
    let jobs = list.len() * kernels.len();
    let next = Arc::new(Mutex::new(0usize));
    let results = Arc::new(Mutex::new(vec![Value::Null; jobs]));
    std::thread::scope(|s| {
        for _ in 0..o.jobs.min(jobs.max(1)) {
            let (next, results) = (Arc::clone(&next), Arc::clone(&results));
            s.spawn(move || {
                loop {
                    let i = {
                        let Ok(mut n) = next.lock() else { return };
                        let i = *n;
                        *n += 1;
                        i
                    };
                    // Case by case, each on every backend in turn.
                    let (Some(case), Some(kernel)) = (list.get(i / kernels.len().max(1)), kernels.get(i % kernels.len().max(1))) else { return };
                    let r = run_one(o, case, kernel);
                    let tag = if kernels.len() > 1 { format!("{kernel:<10}") } else { String::new() };
                    println!(
                        "{:<11} {tag}{}",
                        r["status"].as_str().unwrap_or("?").to_uppercase(),
                        case.strip_prefix(root()).unwrap_or(case).display()
                    );
                    if let Ok(mut all) = results.lock()
                        && let Some(slot) = all.get_mut(i)
                    {
                        *slot = r;
                    }
                }
            });
        }
    });
    let all = Arc::try_unwrap(results).ok().and_then(|m| m.into_inner().ok()).unwrap_or_default();
    (0..kernels.len()).map(|k| all.iter().skip(k).step_by(kernels.len().max(1)).cloned().collect()).collect()
}

/// What went wrong in a result: its failed checks, then its error.
fn notes(r: &Value) -> Vec<String> {
    let mut notes: Vec<String> = r["checks"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|c| !c["ok"].as_bool().unwrap_or(false))
        .map(|c| format!("{} {} (want {})", c["check"].as_str().unwrap_or(""), short(&c["got"]), short(&c["want"])))
        .collect();
    if let Some(e) = r["error"].as_str() {
        notes.push(e.chars().take(160).collect());
    }
    notes
}

pub(crate) fn cell(r: Option<&Value>) -> String {
    let Some(r) = r else { return String::new() };
    let status = r["status"].as_str().unwrap_or("?");
    let notes = notes(r);
    let s = match status {
        "pass" => "pass".to_string(),
        "unsupported" => "not supported yet".to_string(),
        other => format!("**{}**", other.to_uppercase()),
    };
    if notes.is_empty() { s } else { format!("{s}: {}", notes.join("; ")).replace('|', "/").replace('\n', " ") }
}

fn short(v: &Value) -> String {
    match v {
        Value::Number(n) => n.as_f64().map(|x| format!("{x:.6}").trim_end_matches('0').trim_end_matches('.').to_string()).unwrap_or_default(),
        Value::String(s) => s.chars().take(100).collect(),
        Value::Array(a) => format!("[{}]", a.iter().map(short).collect::<Vec<_>>().join(", ")),
        other => other.to_string(),
    }
}

fn sources_of(want: &Value) -> Vec<String> {
    want["sources"].as_array().into_iter().flatten().filter_map(|s| s.as_str().map(str::to_string)).collect()
}

/// The number in `owner/repo#123`, for sorting.
fn source_number(s: &str) -> u64 {
    s.rsplit('#').next().and_then(|n| n.parse().ok()).unwrap_or(u64::MAX)
}

/// `tangency-labs/solidcraft#123` as a link.
fn link(source: &str) -> String {
    match source.split_once('#') {
        Some((repo, n)) if repo.contains('/') => format!("[{source}](https://github.com/{repo}/issues/{n})"),
        _ => source.to_string(),
    }
}

/// Coverage by kind of source (issue, merged PR, a frozen truck-geometry PR...), from
/// `sources.json`: how many were looked at, how many became cases, how many were skipped.
fn source_kinds(dir: &Path, cited: &BTreeMap<String, Vec<String>>) -> Result<String, String> {
    let kinds = read(&dir.join("sources.json"));
    let Some(kinds) = kinds.as_object() else { return Ok(String::new()) };
    let cited: std::collections::BTreeSet<u64> = cited.keys().map(|s| source_number(s)).collect();
    let mut table: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for (n, kind) in kinds {
        let row = table.entry(kind.as_str().unwrap_or("?").to_string()).or_default();
        if n.parse::<u64>().is_ok_and(|n| cited.contains(&n)) {
            row.0 += 1;
        } else {
            row.1 += 1;
        }
    }
    let mut md = String::from("\n## Coverage by source\n\n| Source | Looked at | Became cases | Skipped |\n|---|---|---|---|\n");
    let (mut a, mut b) = (0, 0);
    for (kind, (cases, skipped)) in &table {
        md += &format!("| {kind} | {} | {cases} | {skipped} |\n", cases + skipped);
        a += cases;
        b += skipped;
    }
    md += &format!("| **all** | {} | {a} | {b} |\n", a + b);
    Ok(md)
}

/// The report of one backend's run into `to` (relative to the repository root, or absolute).
fn write_report(dir: &Path, list: &[PathBuf], results: &[Value], kernel: &str, to: &Path) -> Result<String, String> {
    let key = |r: &Value| (r["area"].as_str().unwrap_or("").to_string(), r["case"].as_str().unwrap_or("").to_string());
    let by_case: BTreeMap<(String, String), &Value> = results.iter().map(|r| (key(r), r)).collect();
    // Coverage: per area, cases and their statuses; per source, its cases.
    let mut areas: BTreeMap<String, (usize, StatusCounts)> = BTreeMap::new();
    let mut per_source: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
    let mut rows = String::new();
    for case in list {
        let want = read(&case.join("expected.json"));
        let area = case.parent().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let id = case.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let entry = areas.entry(area.clone()).or_default();
        entry.0 += 1;
        let run = by_case.get(&(area.clone(), id.clone())).copied();
        let status = run.and_then(|r| r["status"].as_str()).unwrap_or("not run").to_string();
        *entry.1.entry(status).or_default() += 1;
        let sources = sources_of(&want);
        for s in &sources {
            per_source.entry(s.clone()).or_default().push(format!("{area}/{id}"));
        }
        *kinds.entry(want["source"].as_str().unwrap_or("?").split_whitespace().next().unwrap_or("?").to_string()).or_default() += 1;
        rows += &format!(
            "| {area}/{id} | {} | {} | {} | {} |\n",
            sources.iter().map(|s| link(s)).collect::<Vec<_>>().join(" "),
            want["what"].as_str().unwrap_or("").replace('|', "/"),
            want["source"].as_str().unwrap_or("?"),
            cell(run)
        );
    }
    let mut md = String::from(
        "# Kernel regression rig\n\nGenerated by `cargo xtask kernel-cases`. Every geometry case the old kernel (truck) got wrong, as an issue, a PR or a crash report recorded it, is a use case in [`tests/kernel-cases/`](../tests/kernel-cases/) with its own right answer: numbers from analysis or a hand calculation (**analytic**) where the geometry has them, and otherwise the invariants (**invariants**): the operation succeeds, or refuses cleanly where that is the right answer; each body is one closed shell that passes the record's structure and geometry checks and SMLib's own check; the expected face, edge and vertex counts. Never a kernel's output, and never a commercial kernel's.\n\nThe cases run on SMLib, SolidCraft's kernel. A failing case stays failing until the kernel is fixed; there is no workaround on SolidCraft's side. A failed SMLib call leaves its inputs for replay in the run's output folder (`target/kernel-cases/dumps/<area>-<case>/`). See [`tests/kernel-cases/README.md`](../tests/kernel-cases/README.md) for the format.\n\n",
    );
    if kernel != "smlib" {
        md += &format!("**This run: `SOLIDCRAFT_KERNEL={kernel}`.**\n\n");
    }
    md += "## Coverage\n\n| Area | Cases | Results |\n|---|---|---|\n";
    for (area, (n, statuses)) in &areas {
        md += &format!("| {area} | {n} | {} |\n", statuses.iter().map(|(s, c)| format!("{c} {s}")).collect::<Vec<_>>().join(", "));
    }
    md += &format!(
        "\n{} cases from {} sources; answers: {}.\n\n## Results\n\n| Case | Sources | What went wrong | Answer | Result |\n|---|---|---|---|---|\n",
        list.len(),
        per_source.len(),
        kinds.iter().map(|(k, v)| format!("{v} {k}")).collect::<Vec<_>>().join(", ")
    );
    md += &rows;
    md += "\n## Sources\n\n| Source | Cases |\n|---|---|\n";
    for (s, cs) in &per_source {
        md += &format!("| {} | {} |\n", link(s), cs.join(", "));
    }
    md += &source_kinds(dir, &per_source)?;
    // What was looked at and skipped, area by area (`<area>/skipped.json`).
    md += "\n## Looked at and skipped\n\n";
    let mut skipped = 0;
    let mut area_dirs: Vec<PathBuf> = std::fs::read_dir(dir).map_err(|e| e.to_string())?.flatten().map(|e| e.path()).filter(|p| p.is_dir()).collect();
    area_dirs.sort();
    for a in area_dirs {
        // skipped.json, or several (skipped-<part>.json) where people triage an area in parts.
        let mut files: Vec<PathBuf> = std::fs::read_dir(&a)
            .map_err(|e| e.to_string())?
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("skipped") && n.ends_with(".json")))
            .collect();
        files.sort();
        let mut items: Vec<Value> = files.iter().flat_map(|f| read(f).as_array().cloned().unwrap_or_default()).collect();
        if items.is_empty() {
            continue;
        }
        items.sort_by_key(|it| source_number(it["source"].as_str().unwrap_or("")));
        let items = &items;
        md += &format!("### {}\n\n| Source | Why skipped |\n|---|---|\n", a.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
        for it in items {
            skipped += 1;
            md += &format!("| {} | {} |\n", link(it["source"].as_str().unwrap_or("?")), it["why"].as_str().unwrap_or("").replace('|', "/"));
        }
        md += "\n";
    }
    if skipped == 0 {
        md += "None yet.\n";
    }
    let path = root().join(to);
    let shown = path.strip_prefix(root()).unwrap_or(&path).display().to_string();
    std::fs::write(&path, md).map_err(|e| format!("{shown}: {e}"))?;
    let pass = results.iter().filter(|r| r["status"] == "pass").count();
    let on = if kernel == "smlib" { String::new() } else { format!(" on {kernel}") };
    Ok(format!("kernel-cases: {pass}/{} pass{on} → {shown} ({skipped} skipped sources listed)", list.len()))
}

/// pass, fail, or other (crash, hang, not supported yet...), for the summary.
fn bucket(status: &str) -> &'static str {
    match status {
        "pass" => "pass",
        "fail" => "fail",
        _ => "other",
    }
}

/// A result's commands' time (or, without one, its process's).
fn seconds(r: &Value) -> Option<f64> {
    r["seconds"].as_f64().or_else(|| r["process_seconds"].as_f64())
}

/// The two backends side by side: per case (area, id) each one's status, time and what went wrong,
/// as JSON, and the Markdown report: a summary, then per area the cases that differ first.
fn side_by_side(smlib: &[Value], solidcore: &[Value]) -> (Value, String) {
    let key = |r: &Value| (r["area"].as_str().unwrap_or("").to_string(), r["case"].as_str().unwrap_or("").to_string());
    let mut pairs: BTreeMap<(String, String), [Option<&Value>; 2]> = BTreeMap::new();
    for (k, rs) in [smlib, solidcore].iter().enumerate() {
        for r in rs.iter().filter(|r| !r.is_null()) {
            if let Some(slot) = pairs.entry(key(r)).or_default().get_mut(k) {
                *slot = Some(r);
            }
        }
    }
    let status = |r: Option<&Value>| r.and_then(|r| r["status"].as_str()).unwrap_or("not run").to_string();
    let mut rows = Vec::new();
    let (mut counts, mut times, mut agree) = ([StatusCounts::new(), StatusCounts::new()], [0.0f64; 2], 0);
    for ((area, case), runs) in &pairs {
        let same = status(runs[0]) == status(runs[1]);
        agree += usize::from(same);
        let mut row = json!({"case": case, "area": area, "agree": same});
        for (k, name) in BACKENDS.iter().enumerate() {
            let r = runs[k];
            *counts[k].entry(bucket(&status(r)).to_string()).or_default() += 1;
            times[k] += r.and_then(seconds).unwrap_or(0.0);
            let error = r.map(|r| notes(r).join("; ")).filter(|e| !e.is_empty());
            row[*name] = json!({"status": status(r), "seconds": r.and_then(seconds), "error": error});
        }
        rows.push(row);
    }
    let differ = pairs.len() - agree;
    let mut md = String::from(
        "# Kernel regression rig: SMLib and solidcore\n\nGenerated by `cargo xtask kernel-cases --kernel both`: every case in [`tests/kernel-cases/`](../tests/kernel-cases/) run on each backend (`SOLIDCRAFT_KERNEL`), graded against its own answer. Times are the commands' own. [`kernel-cases.md`](kernel-cases.md) is the SMLib run in full.\n\n## Summary\n\n| Backend | Pass | Fail | Other | Time (s) |\n|---|---|---|---|---|\n",
    );
    for (k, name) in BACKENDS.iter().enumerate() {
        let n = |b: &str| counts[k].get(b).copied().unwrap_or(0);
        md += &format!("| {name} | {} | {} | {} | {:.1} |\n", n("pass"), n("fail"), n("other"), times[k]);
    }
    md += &format!("\n{} cases: {agree} agree, {differ} differ.\n", pairs.len());
    let side = |r: &Value| match r["seconds"].as_f64() {
        Some(s) => format!("{} ({s:.2} s)", r["status"].as_str().unwrap_or("?")),
        None => r["status"].as_str().unwrap_or("?").to_string(),
    };
    let mut by_area: BTreeMap<&str, Vec<&Value>> = BTreeMap::new();
    for row in &rows {
        by_area.entry(row["area"].as_str().unwrap_or("")).or_default().push(row);
    }
    for (area, mut list) in by_area {
        list.sort_by_key(|r| (r["agree"].as_bool().unwrap_or(false), r["case"].as_str().unwrap_or("").to_string()));
        md += &format!("\n## {area}\n\n| Case | smlib | solidcore | |\n|---|---|---|---|\n");
        for r in list {
            let mark = if r["agree"].as_bool().unwrap_or(false) { "" } else { "**differ**" };
            md += &format!("| {} | {} | {} | {mark} |\n", r["case"].as_str().unwrap_or(""), side(&r["smlib"]), side(&r["solidcore"]));
        }
    }
    let summary: Value = BACKENDS
        .iter()
        .enumerate()
        .map(|(k, name)| (name.to_string(), json!({"counts": counts[k], "seconds": times[k]})))
        .collect::<serde_json::Map<_, _>>()
        .into();
    (json!({"summary": summary, "agree": agree, "differ": differ, "cases": rows}), md)
}

pub fn run(rest: &[&str]) -> Result<(), String> {
    if rest.contains(&"--ladder") {
        if options(rest, "target/kernel-ladder")?.kernels != ["smlib"] {
            return Err("--kernel: the ladder runs on smlib only".into());
        }
        return crate::kernel_ladder::run(rest);
    }
    let o = options(rest, "target/kernel-cases")?;
    let dir = root().join("tests/kernel-cases");
    let list = cases(&dir, o.area.as_deref())?;
    if list.is_empty() {
        return Err(format!("no cases (recipe.json + expected.json) under {}", dir.display()));
    }
    let runs = run_on(&o, &list, &o.kernels);
    for (kernel, results) in o.kernels.iter().zip(&runs) {
        let out = o.out_of(kernel);
        save_json(&out, "results.json", results)?;
        let to = if *kernel == "smlib" { PathBuf::from("docs/kernel-cases.md") } else { out.join("report.md") };
        println!("{}", write_report(&dir, &list, results, kernel, &to)?);
    }
    if let [smlib, solidcore] = runs.as_slice() {
        let (data, md) = side_by_side(smlib, solidcore);
        save_json(&o.out, "backends.json", &data)?;
        std::fs::write(root().join("docs/kernel-cases-backends.md"), md).map_err(|e| format!("docs/kernel-cases-backends.md: {e}"))?;
        println!(
            "kernel-cases: {} agree, {} differ → docs/kernel-cases-backends.md, {}",
            data["agree"],
            data["differ"],
            o.out.join("backends.json").display()
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn side_by_side_counts_and_puts_differences_first() {
        let r = |area: &str, case: &str, status: &str, s: f64| json!({"area": area, "case": case, "status": status, "seconds": s});
        let smlib = [r("booleans", "a", "pass", 1.0), r("booleans", "b", "pass", 2.0), r("shell", "c", "crash", 0.5)];
        let mut hang = json!({"area": "booleans", "case": "b", "status": "hang", "error": "no result after 180 s"});
        hang["process_seconds"] = json!(180.0);
        let solidcore = [r("booleans", "a", "pass", 0.5), hang, r("shell", "c", "fail", 1.5)];
        let (data, md) = side_by_side(&smlib, &solidcore);
        assert_eq!((data["agree"].as_u64(), data["differ"].as_u64()), (Some(1), Some(2)));
        assert_eq!(data["summary"]["smlib"]["counts"], json!({"pass": 2, "other": 1}));
        assert_eq!(data["summary"]["solidcore"]["counts"], json!({"pass": 1, "fail": 1, "other": 1}));
        assert_eq!(data["summary"]["solidcore"]["seconds"].as_f64(), Some(182.0));
        let b = data["cases"].as_array().and_then(|c| c.iter().find(|c| c["case"] == "b")).cloned().unwrap_or_default();
        assert_eq!((b["solidcore"]["status"].as_str(), b["solidcore"]["error"].as_str()), (Some("hang"), Some("no result after 180 s")));
        // In booleans, b (differs) comes before a (agrees).
        let (pos_a, pos_b) = (md.find("| a |"), md.find("| b |"));
        assert!(pos_b.is_some() && pos_a.is_some() && pos_b < pos_a, "{md}");
        assert!(md.contains("| b | pass (2.00 s) | hang (180.00 s) | **differ** |"), "{md}");
        assert!(md.contains("3 cases: 1 agree, 2 differ."), "{md}");
    }
}
