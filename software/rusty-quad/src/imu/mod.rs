use nalgebra::Vector3;

mod imu_variants;

pub struct Dof6ImuData<T> {
    pub acc : Vector3<T>,
    pub mag : Vector3<T>,
}

pub struct Dof9ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
    pub mag : Vector3<T>
}

pub trait Dof6Imu<T> {
    fn read_acc() -> Vector3<T>;
    fn read_gyr() -> Vector3<T>;
    fn read_6dof() -> Dof6ImuData<T>;
}

pub trait Dof9Imu<T> : Dof6Imu<T> {
    fn read_mag() -> Vector3<T>;
    fn read_9dof() -> Dof9ImuData<T>;
}

pub trait AsyncDof6Imu<T> {
    async fn read_acc() -> Vector3<T>;
    async fn read_gyr() -> Vector3<T>;
    async fn read_6dof() -> Dof6ImuData<T>;
}

pub trait AsyncDof9Imu<T> : AsyncDof6Imu<T> {
    async fn read_mag() -> Vector3<T>;
    async fn read_9dof() -> Dof9ImuData<T>;
}
