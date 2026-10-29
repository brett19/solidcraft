//! Which kernel answers each operation: the C++ SMLib fork, or solidcore, its Rust port.
//!
//! [`Op::route`] is the routing table. An operation moves to solidcore only once solidcore
//! matches the C++ kernel on the kernel-case rig (`cargo xtask kernel-cases --kernel both`), and
//! the commit that moves it records the scores. A call that fails on solidcore fails: there is no
//! fallback to C++ at run time, which would hide solidcore's bugs.
//!
//! `SOLIDCRAFT_KERNEL=smlib|solidcore` sends every operation to one kernel for the whole process
//! (the rig, the test matrix); otherwise the preference ([`set_choice`]) does, and its default
//! follows the table.

use std::sync::OnceLock;
use std::sync::atomic::{AtomicU8, Ordering};

/// A kernel an operation runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// The C++ SMLib fork (tangency-labs/smlib) behind its C API.
    Smlib,
    /// solidcore (tangency-labs/solidcore), the Rust port of SMLib.
    Solidcore,
}

impl Backend {
    /// The id `SOLIDCRAFT_KERNEL`, preferences and reports spell it with.
    pub fn id(self) -> &'static str {
        match self {
            Backend::Smlib => "smlib",
            Backend::Solidcore => "solidcore",
        }
    }

    /// Back from an id; anything else is `None`.
    pub fn from_id(id: &str) -> Option<Backend> {
        [Backend::Smlib, Backend::Solidcore].into_iter().find(|b| b.id() == id)
    }
}

/// The operation families the table routes, in the order they move to solidcore.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// A schema 4 record into a kernel body, and back (`from_record`, `to_native`).
    Convert,
    /// Boxes, cylinders, spheres, cones and tori.
    Primitive,
    /// Copies and affine transforms.
    Transform,
    /// Volume, area, mass properties, bounds, counts, distances.
    Measure,
    /// The body checker.
    Check,
    /// Booleans, unions, 2D booleans, imprints.
    Boolean,
    /// Fillets and chamfers, with setbacks and variable radii.
    Blend,
    /// Shell, face offset, thicken, sheet offset.
    Offset,
    /// Split, trim, untrim, extend, merge faces, draft, stitch.
    Split,
    /// Extrusions, revolutions, sweeps and lofts.
    Sweep,
    /// Face and edge history of an operation.
    History,
    /// Surface-surface intersection.
    Ssi,
    /// Sheet metal: flanges and flat patterns.
    Flat,
    /// Native `.smb` files (C++ only: solidcore reads no SMLib file).
    File,
}

impl Op {
    /// Every family, in table order.
    pub const ALL: [Op; 14] = [
        Op::Convert,
        Op::Primitive,
        Op::Transform,
        Op::Measure,
        Op::Check,
        Op::Boolean,
        Op::Blend,
        Op::Offset,
        Op::Split,
        Op::Sweep,
        Op::History,
        Op::Ssi,
        Op::Flat,
        Op::File,
    ];

    /// The routing table: the kernel each family runs on when nothing overrides it. Every family
    /// stays on SMLib until the rig shows solidcore's parity for it.
    pub fn route(self) -> Backend {
        match self {
            Op::Convert
            | Op::Primitive
            | Op::Transform
            | Op::Measure
            | Op::Check
            | Op::Boolean
            | Op::Blend
            | Op::Offset
            | Op::Split
            | Op::Sweep
            | Op::History
            | Op::Ssi
            | Op::Flat
            | Op::File => Backend::Smlib,
        }
    }

    /// The kernel this family runs on now: `SOLIDCRAFT_KERNEL`, else the preference, else
    /// [`Op::route`]. `.smb` files stay on SMLib whatever is chosen.
    pub fn backend(self) -> Backend {
        self.backend_under(choice())
    }

    /// The kernel this family runs on under `c`, whatever is in force now: what [`Op::backend`]
    /// answers once `c` is the choice. `.smb` files stay on SMLib whatever is chosen.
    pub fn backend_under(self, c: Choice) -> Backend {
        if self == Op::File {
            return Backend::Smlib;
        }
        match c {
            Choice::Routed => self.route(),
            Choice::Only(b) => b,
        }
    }
}

/// What answers the operations: the routing table, or one kernel for all of them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Choice {
    /// Each family on its kernel in [`Op::route`].
    #[default]
    Routed,
    /// Every family on this kernel.
    Only(Backend),
}

impl Choice {
    /// `routed`, `smlib` or `solidcore`.
    pub fn id(self) -> &'static str {
        match self {
            Choice::Routed => "routed",
            Choice::Only(b) => b.id(),
        }
    }

    /// Back from [`Choice::id`]; anything else is `None`.
    pub fn from_id(id: &str) -> Option<Choice> {
        if id == "routed" { Some(Choice::Routed) } else { Backend::from_id(id).map(Choice::Only) }
    }
}

/// `SOLIDCRAFT_KERNEL`, read once. An unknown value is no override (and says so in the log).
fn environment() -> Option<Choice> {
    static ENV: OnceLock<Option<Choice>> = OnceLock::new();
    *ENV.get_or_init(|| {
        let v = std::env::var("SOLIDCRAFT_KERNEL").ok()?;
        let c = Choice::from_id(&v);
        if c.is_none() {
            log::warn!(target: "smlib", "SOLIDCRAFT_KERNEL={v}: not smlib, solidcore or routed; ignored");
        }
        c
    })
}

static PREFERENCE: AtomicU8 = AtomicU8::new(0);

/// The choice in force: `SOLIDCRAFT_KERNEL` when it is set, else the preference.
pub fn choice() -> Choice {
    environment().unwrap_or_else(|| match PREFERENCE.load(Ordering::Relaxed) {
        1 => Choice::Only(Backend::Smlib),
        2 => Choice::Only(Backend::Solidcore),
        _ => Choice::Routed,
    })
}

/// Sets the preference. `SOLIDCRAFT_KERNEL`, when set, still wins.
pub fn set_choice(c: Choice) {
    let v = match c {
        Choice::Routed => 0,
        Choice::Only(Backend::Smlib) => 1,
        Choice::Only(Backend::Solidcore) => 2,
    };
    PREFERENCE.store(v, Ordering::Relaxed);
}

/// Whether `SOLIDCRAFT_KERNEL` overrides the preference in this process.
pub fn overridden() -> bool {
    environment().is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_round_trip() {
        for c in [Choice::Routed, Choice::Only(Backend::Smlib), Choice::Only(Backend::Solidcore)] {
            assert_eq!(Choice::from_id(c.id()), Some(c));
        }
        assert_eq!(Choice::from_id("truck"), None);
    }

    #[test]
    fn files_stay_on_smlib() {
        assert_eq!(Op::File.backend(), Backend::Smlib);
    }

    /// Each choice sends the families where it says, read without setting the process-wide
    /// preference: the other tests in this binary build bodies meanwhile, and one flipped to
    /// solidcore for an instant would build theirs there.
    #[test]
    fn each_choice_routes_every_family_without_touching_the_preference() {
        for op in Op::ALL {
            let want = |b: Backend| if op == Op::File { Backend::Smlib } else { b };
            assert_eq!(op.backend_under(Choice::Routed), op.route(), "{op:?}");
            assert_eq!(op.backend_under(Choice::Only(Backend::Smlib)), want(Backend::Smlib), "{op:?}");
            assert_eq!(op.backend_under(Choice::Only(Backend::Solidcore)), want(Backend::Solidcore), "{op:?}");
        }
    }

    /// The preference is what `backend` follows. Only a choice that routes every family as the
    /// table does is set here, so no other test's body changes kernel.
    #[test]
    fn the_preference_routes_unless_the_environment_overrides() {
        if overridden() {
            return;
        }
        let harmless = [Choice::Only(Backend::Smlib), Choice::Only(Backend::Solidcore)]
            .into_iter()
            .find(|c| Op::ALL.iter().all(|op| op.backend_under(*c) == op.route()));
        if let Some(c) = harmless {
            set_choice(c);
            assert_eq!(choice(), c);
            assert!(Op::ALL.iter().all(|op| op.backend() == op.backend_under(c)));
        }
        set_choice(Choice::Routed);
        assert_eq!(choice(), Choice::Routed);
        assert!(Op::ALL.iter().all(|op| op.backend() == op.route()));
    }
}
