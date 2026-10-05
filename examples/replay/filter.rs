//! The filter the harness drives: `Eskf` itself, or, built with `--cfg fusion_nav_onboard`, the
//! `onboard` crate's `Recorder` around it, which writes every call down for #41's board.
//!
//! The default is the filter alone, so every statistic the harness prints rests on `Eskf` and
//! nothing between, and the published crate, which cannot carry the unpublished `onboard`,
//! builds this example. `data/onboard.sh` builds the other, and CI holds the two to the same
//! output byte for byte.

#[cfg(not(fusion_nav_onboard))]
pub use plain::{Filter, Trace};
#[cfg(fusion_nav_onboard)]
pub use traced::{Filter, Trace};

#[cfg(not(fusion_nav_onboard))]
mod plain {
    use std::io;
    use std::ops::{Deref, DerefMut};
    use std::path::Path;

    use fusion_nav::prelude::*;

    /// No trace: this build cannot write one.
    pub enum Trace {}

    impl Trace {
        pub fn create(_: &Path) -> io::Result<Self> {
            Err(io::Error::other(
                "--trace needs the build data/onboard.sh makes, \
                 RUSTFLAGS='--cfg fusion_nav_onboard'",
            ))
        }

        pub fn finish(self) -> io::Result<u64> {
            match self {}
        }
    }

    /// `Eskf`, and the window its start folded, which the harness reads its noise off.
    #[derive(Clone)]
    pub struct Filter {
        eskf: Eskf,
        window: StaticWindow,
    }

    impl Deref for Filter {
        type Target = Eskf;

        fn deref(&self) -> &Eskf {
            &self.eskf
        }
    }

    impl DerefMut for Filter {
        fn deref_mut(&mut self) -> &mut Eskf {
            &mut self.eskf
        }
    }

    impl Filter {
        pub fn new(config: Config, _: Option<Trace>) -> Result<Self, ConfigError> {
            Ok(Self {
                eskf: Eskf::new(config)?,
                window: StaticWindow::new(),
            })
        }

        /// Fold `samples` into a window and start on it.
        pub fn initialize_on(
            &mut self,
            samples: impl IntoIterator<Item = StaticSample>,
        ) -> Result<Alignment, InitError> {
            let mut window = StaticWindow::new();
            for sample in samples {
                window.push(sample)?;
            }
            let alignment = self.eskf.initialize(&window);
            self.window = window;
            alignment
        }

        pub fn window(&self) -> &StaticWindow {
            &self.window
        }

        pub fn take_sink(&mut self) -> Option<Trace> {
            None
        }

        pub fn resume(&mut self, _: Option<Trace>) -> io::Result<()> {
            Ok(())
        }
    }
}

#[cfg(fusion_nav_onboard)]
mod traced {
    use std::ops::{Deref, DerefMut};

    use fusion_nav::prelude::*;
    use onboard::Recorder;
    pub use onboard::TraceFile as Trace;

    /// The `Recorder`, which every call goes through on its way to `Eskf`.
    #[derive(Clone)]
    pub struct Filter(Recorder<Trace>);

    impl Deref for Filter {
        type Target = Recorder<Trace>;

        fn deref(&self) -> &Recorder<Trace> {
            &self.0
        }
    }

    impl DerefMut for Filter {
        fn deref_mut(&mut self) -> &mut Recorder<Trace> {
            &mut self.0
        }
    }

    impl Filter {
        pub fn new(config: Config, trace: Option<Trace>) -> Result<Self, ConfigError> {
            Recorder::new(config, trace).map(Self)
        }
    }
}
