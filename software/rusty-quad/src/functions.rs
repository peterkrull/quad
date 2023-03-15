use core::ops::{Add, Div, Mul, Sub};
use num_traits::float::Float;

pub fn circular<T>(num: T, min: T, max: T) -> T
where
    T: Float,
{
    let frac = ((num - min) / (max - min)).floor();
    num - frac * (max - min)
}

pub fn mapf<T>(num: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T
where
    T: Sub<Output = T> + Add<Output = T> + Mul<Output = T> + Div<Output = T> + Copy,
{
    let out_delta = out_max - out_min;
    let in_delta = in_max - in_min;
    ((num - in_min) / in_delta) * out_delta + out_min
}
