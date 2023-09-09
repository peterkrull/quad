use nalgebra::Vector3;

mod imu_variants;

pub struct Dof6ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
}

pub struct Dof9ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
    pub mag : Vector3<T>
}

pub trait Dof6Imu<T,E> {
    fn read_acc(&mut self) -> Result<Vector3<T>,E>;
    fn read_gyr(&mut self) -> Result<Vector3<T>,E>;
    fn read_6dof(&mut self) -> Result<Dof6ImuData<T>,E>;
}

pub trait Dof9Imu<T,E> : Dof6Imu<T,E> {
    fn read_mag(&mut self) -> Result<Vector3<T>,E>;
    fn read_9dof(&mut self) -> Result<Dof9ImuData<T>,E>;
}

pub trait AsyncDof6Imu<T,E> {
    async fn read_acc(&mut self) -> Result<Vector3<T>,E>;
    async fn read_gyr(&mut self) -> Result<Vector3<T>,E>;
    async fn read_6dof(&mut self) -> Result<Dof6ImuData<T>,E>;
}

pub trait AsyncDof9Imu<T,E> : AsyncDof6Imu<T,E> {
    async fn read_mag(&mut self) -> Result<Vector3<T>,E>;
    async fn read_9dof(&mut self) -> Result<Dof9ImuData<T>,E>;
}