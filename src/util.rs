//! Small iterator helpers.
//!
//! `zip(&mut xs, &ys)` reads more clearly than `xs.iter_mut().zip(ys.iter())`
//! and puts the inputs — and which are read vs mutated — on the `for` line.

use std::fmt::{Debug, Display};

/// Unwrap an `Option` or panic with context — our replacement for the banned
/// `.expect()`.
///
/// Unlike `expect`, the argument is *context for the panic*, not a sentence
/// about expectations, and the `_dbg`/`_with` variants make it easy to fold
/// runtime values (breadcrumbs) into the message.
pub trait OrPanic {
    type A;
    /// Panic with `breadcrumbs` (via `Display`) if the option is `None`.
    fn or_panic(self, breadcrumbs: impl Display) -> Self::A;
    /// Panic with `breadcrumbs` (via `Debug`) if the option is `None`.
    fn or_panic_dbg(self, breadcrumbs: impl Debug) -> Self::A;
    /// Panic with a lazily-built message if the option is `None`.
    fn or_panic_with(self, mk_msg: impl FnOnce() -> String) -> Self::A;
}

impl<A> OrPanic for Option<A> {
    type A = A;
    fn or_panic(self, breadcrumbs: impl Display) -> A {
        match self {
            Some(x) => x,
            None => panic!("{breadcrumbs}"),
        }
    }
    fn or_panic_dbg(self, breadcrumbs: impl Debug) -> A {
        match self {
            Some(x) => x,
            None => panic!("{breadcrumbs:?}"),
        }
    }
    fn or_panic_with(self, mk_msg: impl FnOnce() -> String) -> A {
        match self {
            Some(x) => x,
            None => panic!("{}", mk_msg()),
        }
    }
}

/// Zip two iterables.
pub fn zip<X, Y>(xs: X, ys: Y) -> std::iter::Zip<X::IntoIter, Y::IntoIter>
where
    X: IntoIterator,
    Y: IntoIterator,
{
    xs.into_iter().zip(ys)
}

/// Zip three iterables into flat `(x, y, z)` tuples.
pub fn zip3<X, Y, Z>(xs: X, ys: Y, zs: Z) -> impl Iterator<Item = (X::Item, Y::Item, Z::Item)>
where
    X: IntoIterator,
    Y: IntoIterator,
    Z: IntoIterator,
{
    xs.into_iter().zip(ys).zip(zs).map(|((x, y), z)| (x, y, z))
}

/// Zip four iterables into flat `(x, y, z, w)` tuples.
pub fn zip4<X, Y, Z, W>(xs: X, ys: Y, zs: Z, ws: W) -> impl Iterator<Item = (X::Item, Y::Item, Z::Item, W::Item)>
where
    X: IntoIterator,
    Y: IntoIterator,
    Z: IntoIterator,
    W: IntoIterator,
{
    xs.into_iter().zip(ys).zip(zs).zip(ws).map(|(((x, y), z), w)| (x, y, z, w))
}
