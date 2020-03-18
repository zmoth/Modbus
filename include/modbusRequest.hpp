//
// Created by mateusz on 04.01.2020.
//

#ifndef PROTOCOLCONVERTER_MODBUSREQUEST_HPP
#define PROTOCOLCONVERTER_MODBUSREQUEST_HPP

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <string>
#include <algorithm>

#include "modbusException.hpp"
#include "modbusUtils.hpp"
#include "modbusCell.hpp"

namespace MB
{
class ModbusRequest {
private:
    uint8_t _slaveID;
    utils::MBFunctionCode _functionCode;

    uint16_t _address;
    uint16_t _registersNumber;

    std::vector<ModbusCell> _values;

    /**
     * @brief
     * Constructs Request from raw data
     * @note
     * 1) if CRC = true input data needs to contain 2 CRC bytes on back (used in RS)
     * @note
     * 2) This is private constructor, you need to use fromRaw or fromRawCRC
     * @param inputData - Is vector of bytes that will be be interpreted, whereas
     * based on CRC parameter method performs CRC calculation and throws exception if it is invalid
     * @throws ModbusException
     **/
    explicit ModbusRequest(const std::vector<uint8_t>& inputData, bool CRC = false) noexcept(false);

public:
    /*
     * @description Constructs Request from raw data
     * @params inputData is a vector of bytes that will be interpreted
     * @throws ModbusException
     * */
    static ModbusRequest fromRaw(const std::vector<uint8_t>& inputData) noexcept(false)
    { return ModbusRequest(inputData); }

    /*
     * @description Constructs Request from raw data and checks it's CRC
     * @params inputData is a vector of bytes that will be interpreted
     * @throws ModbusException
     * NOTE: This methods performs CRC check that may throw ModbusException on invalid CRC
     * */
    static ModbusRequest fromRawCRC(const std::vector<uint8_t>& inputData) { return ModbusRequest(inputData, true); }

    /*
     * @description Constructs Request from it's properties
     * @params SlaveId, FunctionCode (utils::MBFunctionCode) , address of first register, number of registers,
     * optional vector of values
     * */
    explicit ModbusRequest(uint8_t slaveId = 0, utils::MBFunctionCode functionCode = static_cast<utils::MBFunctionCode>(0),
                  uint16_t address = 0, uint16_t registersNumber = 0,
                  std::vector<ModbusCell> values = {}) noexcept;

    ModbusRequest(const ModbusRequest &) = default;

    // Returns string representation of object
    [[nodiscard]] std::string toString() const noexcept;
    // Returns raw bytes representation of object, ready for modbus communication
    [[nodiscard]] std::vector<uint8_t> toRaw() const noexcept;

    // Returns function type based on Modbus function code
    [[nodiscard]] utils::MBFunctionType functionType() const noexcept {
        return utils::functionType(_functionCode);
    }
    // Returns register type based on Modbus function code
    [[nodiscard]] utils::MBFunctionRegisters functionRegisters() const noexcept {
        return utils::functionRegister(_functionCode);
    }

    [[nodiscard]] uint8_t slaveID() const {
        return _slaveID;
    }
    [[nodiscard]] utils::MBFunctionCode functionCode() const {
        return _functionCode;
    }
    [[nodiscard]] uint16_t registerAddress() const {
        return _address;
    }
    [[nodiscard]] uint16_t numberOfRegisters() const {
        return _registersNumber;
    }
    [[nodiscard]] const std::vector<ModbusCell> & registerValues() const {
        return _values;
    }

    void setSlaveId(uint8_t slaveId) {
        _slaveID = slaveId;
    }
    void setFunctionCode(utils::MBFunctionCode functionCode) {
        _functionCode = functionCode;
    }
    void setAddress(uint16_t address) {
        _address = address;
    }
    void setRegistersNumber(uint16_t registersNumber) {
        _registersNumber = registersNumber;
        _values.resize(registersNumber);
    }
    void setValues(const std::vector<ModbusCell> &values) {
        _values = values;
    }
};
}


#endif //PROTOCOLCONVERTER_MODBUSREQUEST_HPP
