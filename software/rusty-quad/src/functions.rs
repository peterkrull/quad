use num_traits::float::Float;

pub fn mapf<T : Float>(num: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T {
    let out_delta = out_max - out_min;
    let in_delta = in_max - in_min;
    ((num - in_min) / in_delta) * out_delta + out_min
}
