/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "mpu6500_terminal_serial_handler.hpp"

#include "tap/algorithms/strtok.hpp"
#include "tap/drivers.hpp"

namespace tap::sensors
{
constexpr char Mpu6500TerminalSerialHandler::USAGE[];

#define SUBJECT_BEING_INSPECTED(subjectsBeingInspected, subject) \
    ((subjectsBeingInspected & subject) == subject)

void Mpu6500TerminalSerialHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

bool Mpu6500TerminalSerialHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    char* arg;
    subjectsBeingInspected.reset(
        InspectSubject::ACCEL | InspectSubject::ANGLES | InspectSubject::GYRO |
        InspectSubject::TEMP);
    while (
        (arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine)))
    {
        if (!SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ANGLES) &&
            strcmp(arg, "angle") == 0)
        {
            subjectsBeingInspected.set(InspectSubject::ANGLES);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::GYRO) &&
            strcmp(arg, "gyro") == 0)
        {
            subjectsBeingInspected.set(InspectSubject::GYRO);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ACCEL) &&
            strcmp(arg, "accel") == 0)
        {
            subjectsBeingInspected.set(InspectSubject::ACCEL);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::TEMP) &&
            strcmp(arg, "temp") == 0)
        {
            subjectsBeingInspected.set(InspectSubject::TEMP);
        }
        else if (strcmp(arg, "-h"))
        {
            outputStream << USAGE;
            return false;
        }
    }

    if (!subjectsBeingInspected)
    {
        outputStream << USAGE;
        return !streamingEnabled;
    }

    printHeader(outputStream);

    terminalSerialStreamCallback(outputStream);
    return true;
}

static inline void checkNeedsTab(bool& needsTab, modm::IOStream& outputStream)
{
    if (needsTab)
    {
        outputStream << "\t";
    }
    else
    {
        needsTab = true;
    }
}

static void printFloatVector(modm::IOStream& outputStream, float x, float y, float z)
{
    outputStream.printf(
        "%.2f\t%.2f\t%.2f",
        static_cast<double>(x),
        static_cast<double>(y),
        static_cast<double>(z));
}

void Mpu6500TerminalSerialHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    bool needsTab = false;
    Mpu6500& mpu = drivers->mpu6500;
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ANGLES))
    {
        checkNeedsTab(needsTab, outputStream);
        printFloatVector(outputStream, mpu.getPitch(), mpu.getRoll(), mpu.getYaw());
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::GYRO))
    {
        checkNeedsTab(needsTab, outputStream);
        printFloatVector(outputStream, mpu.getGx(), mpu.getGy(), mpu.getGz());
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ACCEL))
    {
        checkNeedsTab(needsTab, outputStream);
        printFloatVector(outputStream, mpu.getAx(), mpu.getAy(), mpu.getAz());
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::TEMP))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf("%.2f", static_cast<double>(mpu.getTemp()));
    }
    outputStream << modm::endl;
}

void Mpu6500TerminalSerialHandler::printHeader(modm::IOStream& outputStream)
{
    bool needsTab = false;
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ANGLES))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "pit\trol\tyaw";
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::GYRO))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "gx\tgy\tgz";
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::ACCEL))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "ax\tay\taz";
    }
    if (SUBJECT_BEING_INSPECTED(subjectsBeingInspected, InspectSubject::TEMP))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "temp";
    }
    outputStream << modm::endl;
}
}  // namespace tap::sensors
