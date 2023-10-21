use alloc::boxed::Box;

pub struct Sequencer<S> {
    state: S,
    to: Option<usize>,
    from: Option<usize>,
}

#[derive(Debug, PartialEq)]
enum Event {
    Jump,
    Return,
}

trait State: core::fmt::Debug {
    fn process(&self, event: Event) -> Box<Self>;
}

#[derive(Debug, PartialEq)]
pub struct Free;
#[derive(Debug, PartialEq)]
pub struct Jump { to: usize }
#[derive(Debug, PartialEq)]
pub struct Retrig { to: usize, from: usize }
#[derive(Debug, PartialEq)]
pub struct Hold;
#[derive(Debug, PartialEq)]
pub struct Return;

impl State for Free {
    fn process(&self, event: Event) -> Box<State> {
        match event {
            Event::Jump => Box::new(Hold),
            Event::Return => Box::new(Free),
        }
    }
}
impl State for Jump {}
impl State for Retrig {}
impl State for Hold {}
impl State for Return {}

impl Sequencer<Free> {
    fn new() -> Self {
        Self{ state: Free, to: None, from: None }
    }
}

impl From<Sequencer<Free>> for Sequencer<Jump> {
    fn from(&self, state: Sequencer<Free>) -> Self {
        Self{ state: Jump { to: 0 }, to: self.to, from: self.from}
    }
}

impl From<Sequencer<Free>> for Sequencer<Retrig> {
    fn from(&self, state: Sequencer<Free>) -> Self {
        Self {
            state: Retrig{to: self.to, from: self.from},
            to: None,
            from: None,
        }
    }
}