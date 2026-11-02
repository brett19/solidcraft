//! Cooperative cancellation of solidcore calls (solidcore's `CancelToken`, solidcore#2136).
//!
//! A job that can be stopped (a feature computed off the window thread, a design opening) owns a
//! [`Cancel`] and runs its work inside [`cancelling`]. Every solidcore call that work makes on this
//! thread then runs with the job's token on the kernel, and only that call: the process has one
//! kernel ([`crate::core`]), its calls one at a time under its lock, so a token installed for the
//! length of one call never reaches another job's call. Once the job is cancelled the call running
//! returns at solidcore's next check, and every later call of the job fails at once without
//! running, with [`crate::Error::Cancelled`].
//!
//! A cancelled call's result is never kept: a cache filled lazily and shared with other jobs (a
//! body's conversion between the kernels) is filled under [`shielded`], so a cancel can't leave a
//! "cancelled" answer in it for a job that was not cancelled.
//!
//! The C++ kernel has no cancellation: its calls run to the end as before.

use std::cell::RefCell;

use solidcore::kernel::CancelToken;

/// A job's stop switch. Clones share it; a new job takes a new one, so a cancel never outlives
/// its job.
#[derive(Clone, Debug, Default)]
pub struct Cancel(CancelToken);

impl Cancel {
    pub fn new() -> Cancel {
        Cancel::default()
    }

    /// Stop the job: the solidcore call running returns at its next check, and later ones at once.
    pub fn cancel(&self) {
        self.0.cancel();
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.is_cancelled()
    }
}

thread_local! {
    static CURRENT: RefCell<Option<Cancel>> = const { RefCell::new(None) };
}

/// Run `f` with `cancel` stopping the solidcore calls it makes on this thread (`None`: calls that
/// can't be stopped, as without a job).
pub fn cancelling<R>(cancel: Option<Cancel>, f: impl FnOnce() -> R) -> R {
    let previous = CURRENT.with(|c| c.replace(cancel));
    struct Restore(Option<Option<Cancel>>);
    impl Drop for Restore {
        fn drop(&mut self) {
            if let Some(previous) = self.0.take() {
                CURRENT.with(|c| *c.borrow_mut() = previous);
            }
        }
    }
    let _restore = Restore(Some(previous));
    f()
}

/// Run `f` so that no cancel stops it: for a result kept beyond the job (a lazily filled cache).
pub fn shielded<R>(f: impl FnOnce() -> R) -> R {
    cancelling(None, f)
}

/// The stop switch over this thread's calls, to hand to the worker threads it starts.
pub fn current() -> Option<Cancel> {
    CURRENT.with(|c| c.borrow().clone())
}

/// The token to put on the kernel for a call made now on this thread.
pub(crate) fn token() -> Option<CancelToken> {
    CURRENT.with(|c| c.borrow().as_ref().map(|c| c.0.clone()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cancel_reaches_only_the_calls_made_inside_its_scope() {
        let job = Cancel::new();
        assert!(token().is_none());
        cancelling(Some(job.clone()), || {
            assert!(token().is_some_and(|t| !t.is_cancelled()));
            job.cancel();
            assert!(token().is_some_and(|t| t.is_cancelled()));
            assert!(shielded(token).is_none(), "a shielded fill runs with no token");
            assert!(token().is_some_and(|t| t.is_cancelled()), "the job's token is back after the shield");
        });
        assert!(token().is_none(), "the scope ends with its job");
        assert!(cancelling(Some(Cancel::new()), || token().is_some_and(|t| !t.is_cancelled())), "a new job starts uncancelled");
    }
}
