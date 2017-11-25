#include <cstdint>
#include <type_traits>
#include <initializer_list>
#include <tuple>

// Helper functions
template<std::size_t... S>
constexpr std::size_t sum()
{
  std::size_t result = 0;

  for (auto s : {S...})
    result += s;

  return result;
}

template <bool... >
struct bool_pack {};

template< bool... Bs>
constexpr bool all_true()
{
  return std::is_same< bool_pack< true, Bs... >,
                       bool_pack< Bs..., true > >::value;
}


namespace states
{
// Core states definitions
enum class nominal
{
  position,
  velocity,
  attitude,
  bias_acc,
  bias_gyro
};

enum class error
{
  position,
  velocity,
  attitude,
  bias_acc,
  bias_gyro
};
}

// Tag to easily detect the base class
struct sensor_base_tag {};

// A sensor needs to include ...
template <std::size_t MEASUREMENT_SIZE,
          std::size_t NUM_LINEAR_STATES,
          std::size_t NUM_ROTATION_STATES>
struct sensor_base : sensor_base_tag
{
  using measurement_vector_size =
      std::integral_constant< std::size_t, MEASUREMENT_SIZE >;
  using num_linear_states =
      std::integral_constant< std::size_t, NUM_LINEAR_STATES >;
  using num_rotation_states =
      std::integral_constant< std::size_t, NUM_ROTATION_STATES >;
};

// msf2 specs, takes a list of unique sensors
template <typename... Sensors>
struct msf2_specs
{
  // Check that input list is actually sensors
  static_assert(all_true< std::is_base_of<sensor_base_tag, Sensors>::value ... >(),
                "Not all elements in the sensor list is a sensor, did you forget to "
                "inherit from sensor_base?");

  // Define state sizes
  using num_core_states         = std::integral_constant< std::size_t, 16 >;
  using num_core_error_states   = std::integral_constant< std::size_t, 15 >;
  using num_sensor_states       = std::integral_constant<
                                    std::size_t,
                                    sum<Sensors::num_linear_states::value...>() +     // Get num linear state
                                    4 * sum<Sensors::num_rotation_states::value...>() // Get num rotational states
                                  >;
  using num_sensor_error_states = std::integral_constant<
                                    std::size_t,
                                    sum<Sensors::num_linear_states::value...>() +     // Get num linear state
                                    3 * sum<Sensors::num_rotation_states::value...>() // Get num rotational states
                                  >;

  // Storage for sensors with state
  using sensor_storage_t = std::tuple< Sensors... >;
};

template <typename Spec>
class msf2
{
private:
  typename Spec::sensor_storage_t sensor_storage;

public:
  // Convenience functions to get parts of state vector
  template <typename T>
  constexpr std::size_t get()
  {
    static_assert(
        std::is_base_of< sensor_base_tag, T >::value,
        "Only sensors and state enums can be an argument in get< ... >().");
    static_assert(T::num_linear_states::value > 0,
                  "Can't get sensor state, this sensor has no extra linear "
                  "states defined.");

    return 9001;
  }

  template < typename T >
  constexpr std::size_t get_rot()
  {
    static_assert(
        std::is_base_of< sensor_base_tag, T >::value,
        "Only sensors and state enums can be an argument in get< ... >().");
    static_assert(T::num_rotation_states::value > 0,
                  "Can't get sensor state, this sensor has no extra rotation "
                  "states defined.");

    return 10001;
  }

  template < states::nominal T >
  constexpr std::size_t get()
  {
    return 1;
  }

  template < states::error T >
  constexpr std::size_t get()
  {
    return 2;
  }
};




// Make some sensors (one file/sensor, not made here)
// Fake sensor, just to make it happy
using sensor1 = sensor_base<1,  // Measurement size (1)
                            0,  // Number of extra linear states
                            0>; // Number of extra rotational states
                                //              (1 state = 1 extra quaternion)
using sensor2 = sensor_base<3,
                            1,
                            0>;

using sensor3 = sensor_base<1,
                            1,
                            1>;

// Create an MSF specification from it
using spec = msf2_specs< sensor1, sensor2, sensor3 >;

// Create the MSF2 filter
using msf = msf2< spec >;

#include <iostream>

int main()
{
  msf my_msf;

  // Try get stuff
  std::cout << "val : " << my_msf.get_rot< sensor3 >() << "\n";
  std::cout << "val2: " << sizeof(sensor3) << "\n";

  return 0;
}
