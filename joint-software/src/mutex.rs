use core::{
    cell::UnsafeCell,
    sync::atomic::AtomicBool,
    sync::atomic::Ordering::*,
    future::poll_fn,
    task::Poll,
    ops::{Deref, DerefMut},
    };

pub struct BusyMutex<T> {
    value: UnsafeCell<T>,
    locked: AtomicBool,
}
impl<T> BusyMutex<T> {
    pub fn new(value: T) -> Self {
        Self {
            value: value.into(), 
            locked: AtomicBool::new(false),
        }
    }
    pub async fn lock(&self) -> BusyMutexGuard<'_, T> {
        BusyMutexGuard::new(self).await
    }
}

pub struct BusyMutexGuard<'m, T> {
    mutex: &'m BusyMutex<T>,
}
impl<'m, T> BusyMutexGuard<'m, T> {
    async fn new(mutex: &'m BusyMutex<T>) -> Self {
        poll_fn(|_| 
            if mutex.locked.swap(true, Acquire)
            {Poll::Pending} else {Poll::Ready(())}
            ).await;
        Self {mutex}
    }
}
impl<T> Deref for BusyMutexGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe {& *self.mutex.value.get()}
    }
}
impl<T> DerefMut for BusyMutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe {&mut *self.mutex.value.get()}
    }
}
impl<T> Drop for BusyMutexGuard<'_, T> {
    fn drop(&mut self) {
        self.mutex.locked.store(false, Release);
    }
}
