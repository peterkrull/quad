use embedded_hal_async::i2c::I2c;
use icm20948_async::{Icm20948, Init, IcmError, MagEnabled, MagDisabled};
use nalgebra::Vector3;

// struct Icm20948_async<I2C,MAG,INIT>(Icm20948<I2C,MAG,INIT>);

use crate::imu::{AsyncDof9Imu, AsyncDof6Imu, Dof6ImuData, Dof9ImuData};

impl<I2C,E> AsyncDof6Imu<f32,E> for Icm20948<I2C,MagDisabled,Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    async fn read_acc(&mut self) -> Result<Vector3<f32>,E> {
        icm20948_async::Icm20948::read_acc(self).await
    }

    async fn read_gyr(&mut self) -> Result<Vector3<f32>,E> {
        icm20948_async::Icm20948::read_gyr(self).await
    }

    async fn read_6dof(&mut self) -> Result<Dof6ImuData<f32>,E> {

        let reading = self.read_all().await?;
        Ok(Dof6ImuData{
            acc: reading.acc,
            gyr: reading.gyr,
        })
    }
}

impl<I2C,E> AsyncDof6Imu<f32,E> for Icm20948<I2C,MagEnabled,Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    async fn read_acc(&mut self) -> Result<Vector3<f32>,E> {
        icm20948_async::Icm20948::read_acc(self).await
    }

    async fn read_gyr(&mut self) -> Result<Vector3<f32>,E> {
        icm20948_async::Icm20948::read_gyr(self).await
    }

    async fn read_6dof(&mut self) -> Result<Dof6ImuData<f32>,E> {

        let reading = self.read_all().await?;
        Ok(Dof6ImuData{
            acc: reading.acc,
            gyr: reading.gyr,
        })
    }
}

impl<I2C,E> AsyncDof9Imu<f32,E> for Icm20948<I2C,MagEnabled,Init> 
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>
{
    async fn read_mag(&mut self) -> Result<Vector3<f32>,E> {
        self.read_mag().await
    }

    async fn read_9dof(&mut self) -> Result<Dof9ImuData<f32>,E> {
        let reading = self.read_all().await?;
        Ok(Dof9ImuData{
            acc: reading.acc,
            gyr: reading.gyr,
            mag: reading.mag,
        })
    }
}