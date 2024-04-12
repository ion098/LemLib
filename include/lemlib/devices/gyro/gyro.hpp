#pragma once

#include <cstdint>
#include <memory>
#include <math.h>

namespace lemlib {

class AnyGyro {
    private:
        struct IGyro;
       template <typename T> struct TypeErasedGyro;
    public:
        template <typename T>
        AnyGyro(TypeErasedGyro<T> gyro) : poly_gyro(new TypeErasedGyro<T>(std::move(gyro))) {}
    private:
        std::unique_ptr<IGyro> poly_gyro;
        struct IGyro {
            /**
            * @brief Calibrate the gyro
            *
            * @param blocking whether the function should block until calibration is complete
            * @return true calibration failed
            * @return false calibration succeeded
            */
            virtual bool calibrate(bool blocking = false) = 0;
            /**
            * @brief Get wether the gyro is calibrating or not
            *
            * @return true Gyro is calibrating
            * @return false Gyro is not calibrating
            */
            virtual bool isCalibrating() const = 0;
            /**
            * @brief Get wether the gyro is calibrated or not
            *
            * @return true the gyro is calibrated
            * @return false the gyro is not calibrated
            */
            virtual bool isCalibrated() = 0;
            /**
            * @brief Get whether the gyro is connected or not
            *
            * @return true the gyro is connected
            * @return false the gyro is not connected
            */
            virtual bool isConnected() = 0;
            /**
            * @brief Get the heading of the gyro
            *
            * @note 0 is in the positive x direction, and heading increases counterclockwise
            *
            * @return float heading, in radians, locked from -pi to +pi
            */
            virtual float getHeading() = 0;
            /**
            * @brief Get the rotation of the gyro
            *
            * @note 0 is in the positive x direction, and heading increases counterclockwise
            *
            * @return float rotation, in radians
            */
            virtual float getRotation() = 0;
            /**
            * @brief Set the rotation of the gyro
            *
            * @note 0 is in the positive x direction, and heading increases counterclockwise
            *
            * @param rotation, rotation in radians
            */
            virtual void setRotation(float rotation) const = 0;
            /**
            * @brief Get the change in rotation of the gyro
            *
            * @note positive change is counterclockwise, negative change is clockwise
            *
            * @param update whether to update the last angle measured by the gyro. True by default
            * @return change in angle rotated by the encoder, in radians
            */
            float getRotationDelta(bool update = true);
        };

        template <typename T>
        struct TypeErasedGyro : IGyro {
            T actual_gyro;
            TypeErasedGyro(T _actual_gyro): actual_gyro(_actual_gyro) {}
        };
};

class Gyro {
    public:
        /**
         * @brief Calibrate the gyro
         *
         * @param blocking whether the function should block until calibration is complete
         * @return true calibration failed
         * @return false calibration succeeded
         */
        virtual bool calibrate(bool blocking = false) = 0;
        /**
         * @brief Get wether the gyro is calibrating or not
         *
         * @return true Gyro is calibrating
         * @return false Gyro is not calibrating
         */
        virtual bool isCalibrating() const = 0;
        /**
         * @brief Get wether the gyro is calibrated or not
         *
         * @return true the gyro is calibrated
         * @return false the gyro is not calibrated
         */
        virtual bool isCalibrated() = 0;
        /**
         * @brief Get whether the gyro is connected or not
         *
         * @return true the gyro is connected
         * @return false the gyro is not connected
         */
        virtual bool isConnected() = 0;
        /**
         * @brief Get the heading of the gyro
         *
         * @note 0 is in the positive x direction, and heading increases counterclockwise
         *
         * @return float heading, in radians, locked from -pi to +pi
         */
        virtual float getHeading() = 0;
        /**
         * @brief Get the rotation of the gyro
         *
         * @note 0 is in the positive x direction, and heading increases counterclockwise
         *
         * @return float rotation, in radians
         */
        virtual float getRotation() = 0;
        /**
         * @brief Set the rotation of the gyro
         *
         * @note 0 is in the positive x direction, and heading increases counterclockwise
         *
         * @param rotation, rotation in radians
         */
        virtual void setRotation(float rotation) const = 0;
        /**
         * @brief Get the change in rotation of the gyro
         *
         * @note positive change is counterclockwise, negative change is clockwise
         *
         * @param update whether to update the last angle measured by the gyro. True by default
         * @return change in angle rotated by the encoder, in radians
         */
        float getRotationDelta(bool update = true);
        /**
         * @brief Get the port of the gyro
         *
         * @return std::uint8_t unsigned port of the gyro
         */
        virtual std::uint8_t getPort() = 0;
    protected:
        float lastAngle = M_PI_2;
};
} // namespace lemlib