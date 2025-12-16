# Unit Tests for BasicCarGame

This directory contains comprehensive unit tests for all classes in the BasicCarGame project.

## Test Files

- **RigidBodyTest.cpp** - Tests for the RigidBody base class
  - Constructor initialization
  - Position getters
  - Force and torque accumulation
  - Time integration and physics updates
  - Edge cases (zero mass, multiple time steps)

- **WheelTest.cpp** - Tests for the Wheel class
  - Constructor initialization
  - Linear/angular velocity conversions
  - Friction calculations with various conditions
  - Torque application
  - Time integration
  - Different wheel angles and car orientations

- **CarTest.cpp** - Tests for the Car class
  - Constructor and wheel initialization
  - Steering logic and clamping
  - Force feedback (steering return to center)
  - Engine torque application
  - Braking system
  - Wheel force summation
  - Integration tests
  - Edge cases

## Building and Running Tests

### Prerequisites
- CMake 3.10 or higher
- C++17 compatible compiler
- Internet connection (for first build to download Google Test)

### Build Tests

From the project root directory:

```bash
# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build tests
make RunAllTests

# Or build everything
make
```

### Run Tests

```bash
# From the build directory
./tests/RunAllTests

# Or use CTest for detailed output
ctest --verbose
```

### Run Specific Tests

```bash
# Run only RigidBody tests
./tests/RunAllTests --gtest_filter=RigidBodyTest.*

# Run only Wheel tests
./tests/RunAllTests --gtest_filter=WheelTest.*

# Run only Car tests
./tests/RunAllTests --gtest_filter=CarTest.*

# Run a specific test
./tests/RunAllTests --gtest_filter=CarTest.ApplySteeringClampsAtMaximum
```

## Test Coverage

The tests cover:
- ✅ All public methods in RigidBody, Wheel, and Car classes
- ✅ Edge cases (zero values, negative values, extreme values)
- ✅ Physics calculations and time integration
- ✅ Known bugs (tests document current behavior even if incorrect)
- ✅ Integration scenarios (multiple operations in sequence)

## Known Issues Documented by Tests

Some tests document current behavior that may not be correct:
1. Torque calculation in `sumWheelForces()` uses a generic distance instead of actual wheel positions
2. Y-axis is flipped in position calculations (documented in tests)
3. Some physics constants may need tuning for realistic behavior

These tests will help ensure any fixes don't break other functionality.

## Adding New Tests

To add new tests:
1. Add test cases to the appropriate test file
2. Follow the existing naming pattern: `TEST_F(ClassName, MethodNameDescriptiveTest)`
3. Rebuild: `make RunAllTests`
4. Run: `./tests/RunAllTests`

## Google Test Documentation

For more information on Google Test features:
- Assertions: https://google.github.io/googletest/reference/assertions.html
- Advanced features: https://google.github.io/googletest/advanced.html