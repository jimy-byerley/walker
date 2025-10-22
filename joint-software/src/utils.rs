
/// convert a nb task into a future
#[macro_export]
macro_rules! nb_task {
    ($expression:expr) => {
        core::future::poll_fn(|context| match ($expression) {
            Ok(result) => core::task::Poll::Ready(Ok(result)),
            Err(nb::Error::WouldBlock) => {
                context.waker().clone().wake();
                core::task::Poll::Pending
                },
            Err(error) => core::task::Poll::Ready(Err(error)),
            })
    };
}
