//! Progress and cooperative cancellation for long kernel work (reading a large STEP file) run
//! off the window thread. The caller installs a [`Watch`] around the work with [`watching`];
//! the work reports how far it is and stops at its next check once the watch says so. Work
//! that nobody watches runs as before.
//!
//! A watch can also stop solidcore mid-operation ([`Watch::kernel_cancel`]): a round, an offset, a
//! shell, a sweep or a boolean that takes a minute returns at solidcore's next check once the
//! watch is cancelled, with [`crate::KernelError::Cancelled`] or an error that says "cancelled".

use std::cell::RefCell;
use std::sync::Arc;

/// Someone waiting for long work: told how far it is, and able to stop it.
pub trait Watch: Send + Sync {
    /// Stop at the next check.
    fn cancelled(&self) -> bool;
    /// `done` of `total` units of the current step (`stage`, such as "reading" or "converting").
    fn progress(&self, stage: &'static str, done: usize, total: usize);
    /// The stop switch of the solidcore calls made under this watch, cancelled with it; `None`:
    /// solidcore runs each call to its end.
    fn kernel_cancel(&self) -> Option<Cancel> {
        None
    }
}

pub use solidcraft_smlib::cancel::{Cancel, shielded};

thread_local! {
    static CURRENT: RefCell<Option<Arc<dyn Watch>>> = const { RefCell::new(None) };
}

/// Run `f` with `watch` seeing the kernel work it does on this thread.
pub fn watching<R>(watch: Arc<dyn Watch>, f: impl FnOnce() -> R) -> R {
    let cancel = watch.kernel_cancel();
    let previous = CURRENT.with(|c| c.replace(Some(watch)));
    struct Restore(Option<Option<Arc<dyn Watch>>>);
    impl Drop for Restore {
        fn drop(&mut self) {
            if let Some(previous) = self.0.take() {
                CURRENT.with(|c| *c.borrow_mut() = previous);
            }
        }
    }
    let _restore = Restore(Some(previous));
    solidcraft_smlib::cancel::cancelling(cancel, f)
}

/// The solidcore stop switch over this thread's calls, to hand to the worker threads it starts
/// with [`within`].
pub(crate) fn kernel_cancel() -> Option<Cancel> {
    solidcraft_smlib::cancel::current()
}

/// Run `f` (on a worker thread) under the solidcore stop switch `cancel` of the thread that
/// started it.
pub(crate) fn within<R>(cancel: Option<Cancel>, f: impl FnOnce() -> R) -> R {
    solidcraft_smlib::cancel::cancelling(cancel, f)
}

/// The watch over this thread's work, to hand to the worker threads it starts.
pub fn current() -> Option<Arc<dyn Watch>> {
    CURRENT.with(|c| c.borrow().clone())
}

/// The error long work returns when its watch cancelled it.
pub(crate) fn cancelled_error(what: &str) -> crate::KernelError {
    crate::KernelError::Cancelled(what.to_string())
}
