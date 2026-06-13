//! Small iterator helpers.
//!
//! `zip(&mut xs, &ys)` reads more clearly than `xs.iter_mut().zip(ys.iter())`
//! and puts the inputs — and which are read vs mutated — on the `for` line.

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
