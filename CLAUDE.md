# CLAUDE.md - Guide for Agentic Coding in spn-ipu

## Build & Test Commands
- Build: `cmake -B build && cmake --build build`
- Run all tests: `cd build && ctest`
- Run a single test: `cd build && ctest -R <test_name>`
- Lint: `clang-format -i <file>` (Google style)

## Code Style Guidelines
- C++20 standard with Google style as base
- Namespaces: Use `spnipu` namespace for all code
- Class/Type Naming: PascalCase (e.g., `SumNode`, `GaussianLeafNode`)
- Methods/Functions: camelCase (e.g., `addSummand`, `getFactors`)
- Private members: camelCase with trailing underscore (e.g., `summands_`)
- Error handling: Use logging via spdlog for tracing and debugging
- Imports: Group standard library, then project headers, then external
- Templates: Use template patterns when possible for compile-time correctness
- Use modern C++ features like spans, ranges, and concepts when appropriate

## Project overview
- SPN (Sum-Product Networks) are parsed from Captnproto files, scheduled and then lowered to GraphCore IPUs using the Graphene framework.
- Graphene is a layer (DSL) between the IPU's poplar framework and the user code. Graphene consists of TensorDSL and CodeDSL.
- IPUs are programmed in the Poplar Framework by constructing a dataflow graph and a execution schedule. This is hidden from the user by Graphene.

## Project Structure
- Main library code in `libspnipu/libspnipu/`
- Graphene library code in `libspnipu/libs/graphene/`
- Applications in `apps/`
- Poplar Framework in `/poplar/poplar_sdk-ubuntu_20_04-3.4.0+1507-69d9d03fd8/poplar-ubuntu_20_04-3.4.0+73-aa67dd6164/include/poplar/`
  - A good entry point are the `Graph.hpp` and `Tensor.hpp` files