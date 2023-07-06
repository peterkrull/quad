use icm20948_async::Icm20948;

use crate::imu::{AsyncDof9Imu, AsyncDof6Imu};

impl<I2C,MAG,INIT> AsyncDof6Imu<f32> for Icm20948<I2C,MAG,INIT> {
    async fn read_acc() -> nalgebra::Vector3<f32> {
        todo!()
    }

    async fn read_gyr() -> nalgebra::Vector3<f32> {
        todo!()
    }

    async fn read_6dof() -> crate::imu::Dof6ImuData<f32> {
        todo!()
    }
}

impl<I2C,MAG,INIT> AsyncDof9Imu<f32> for Icm20948<I2C,MAG,INIT> {
    async fn read_mag() -> nalgebra::Vector3<f32> {
        todo!()
    }

    async fn read_9dof() -> crate::imu::Dof9ImuData<f32> {
        todo!()
    }
}