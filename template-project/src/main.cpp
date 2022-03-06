#include "tap/algorithms/smooth_pid.hpp"
#include "tap/board/board.hpp"

#include "drivers_singleton.hpp"

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId MOTOR_ID2 = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId MOTOR_ID3 = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId MOTOR_ID4 = tap::motor::MOTOR4;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM = 3000;

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
tap::algorithms::SmoothPid pidController(20, 0, 0, 0, 8000, 1, 0, 1, 0);
tap::motor::DjiMotor motor(::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor2(::DoNotUse_getDrivers(), MOTOR_ID2, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor3(::DoNotUse_getDrivers(), MOTOR_ID3, CAN_BUS, false, "cool motor");
tap::motor::DjiMotor motor4(::DoNotUse_getDrivers(), MOTOR_ID4, CAN_BUS, false, "cool motor");

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    ::Drivers *drivers = ::DoNotUse_getDrivers();

    Board::initialize();
    drivers->can.initialize();
    motor.initialize();
    motor2.initialize();
    motor3.initialize();
    motor4.initialize();

    while (1)
    {
        if (sendMotorTimeout.execute())
        {
            pidController.runControllerDerivateError(DESIRED_RPM - motor.getShaftRPM(), 1);
            motor.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            motor2.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            motor3.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            motor4.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            drivers->djiMotorTxHandler.processCanSendData();
            //drivers->leds.set(tap::gpio::Leds::A, !motor.isMotorOnline());
        }

        drivers->canRxHandler.pollCanData();
        modm::delay_us(10);
    }
    return 0;
}
