//! Common types and functions.

/// Contains width and height of a frame.
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct Resolution {
    pub width: usize,
    pub height: usize,
}
