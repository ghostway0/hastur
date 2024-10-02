#include <cstdint>
#include <set>
#include <variant>

#include "wasm/instructions.h"
#include "wasm/wasm.h"

#include "regalloc.h"

struct MemoryAddressing {
    Register base;
    Register index;
    int32_t offset;
    int32_t scale;
};

// first liveness analysis, then register allocation.
// liveness would be done by a function that takes a wasm::CodeEntry and returns a std::vector<LiveBundle>. can be done
// by a simple linear scan because wasm is a stack machine. I don't quite understand how I can do this in a fun way; how
// will partial liveness be handled? I can just say the value still lives, but that's not very fun. saying the value is
// dead is incorrect. the regalloc would get its constraints by probing the backend for them on the wasm module.
// std::set<Constraint> get_constraints(wasm::Instruction const &instr); which would, for example for a call with two
// arguments, return {Constraint::Register{RDI}, Constraint::Register{RSI}}. the regalloc would then be run on the
// LiveBundle and the constraints, and return a Solution with the allocations, and stitches. the stitches would be used
// to insert mov instructions to move values between registers. some instructions can be inserted that interact with the
// VMState implicit parameter.

struct Anywhere {};

using Constraint = std::variant<Register, StackSlot, RegClass, Anywhere>

class Backend {
public:
    virtual ~Backend() = default;

    virtual std::set<Constraint> get_constraints(wasm::Instruction const &instr) = 0;
};

class X86Backend : public Backend {
    static constexpr Register kRDI{RegClass::Int, 0};
    static constexpr Register kRSI{RegClass::Int, 1};
    static constexpr Register kRDX{RegClass::Int, 2};
    static constexpr Register kRCX{RegClass::Int, 3};
    static constexpr Register kR8{RegClass::Int, 8};
    static constexpr Register kR9{RegClass::Int, 9};

    static constexpr std::array<Register, 6> kArgumentRegisters = {kRDI, kRSI, kRDX, kRCX, kR8, kR9};

public:
    std::vector<Constraint> get_constraints(wasm::Instruction const &instr) override {
        if (std::holds_alternative<wasm::instructions::Call>(instr)) {
            wasm::instructions::Call const &call = std::get<wasm::instructions::Call>(instr);

            std::vector<Constraint> constraints;
            constraints.reserve(call.arguments.size());

            for (size_t i = 0; i < call.arguments.size(); i++) {
                constraints.push_back(kArgumentRegisters[i]);
            }

            return constraints;
        }

        // FIXME: we probably won't pattern match for instructions that calculate addresses.
        // we also probably would need to get the memory address from the VMState.
        if (std::holds_alternative<wasm::instructions::Load>(instr)) {
            wasm::instructions::Load const &load = std::get<wasm::instructions::Load>(instr);

            return {RegClass::Int};
        }

        return {};
    }
};

int main() {
    Backend *backend = new X86Backend{};
    std::vector<Constraint> constraints = backend->get_constraints(wasm::instructions);
}
