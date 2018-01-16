#include <cstdint>
#include <type_traits>
#include <initializer_list>
#include <tuple>

#include <kvasir/mpl/mpl.hpp>

// Helper functions
namespace mpl = kvasir::mpl;

template < typename Comp = mpl::is_same<>, typename IsSame = mpl::is_same<>,
           typename C = mpl::identity >
using distinct =
    mpl::fork< mpl::size<>,               // Fork 1: size of input
               mpl::stable_sort<          // Fork 2: size of unique list
                   Comp,                  // Comparison for sorting the list
                   mpl::remove_adjacent<  // Remove non unique items
                       IsSame,            // Comparison for uniqueness
                       mpl::size<>        // Get the size
                       > >,
               mpl::is_same<  // Check if the sizes are the same
                   C          // Continuation
                   > >;

namespace eager
{
template < typename List, typename Comp = mpl::is_same<>,
           typename IsSame = mpl::is_same<> >
using distinct = mpl::call< mpl::unpack< distinct< Comp, IsSame > >, List >;
}

void test()
{
  // distinct2<int, float, int, double, char>::g;
  eager::distinct< mpl::list< int, float, int, double, char > >::g;
}

template < std::size_t... S >
constexpr std::size_t sum()
{
  std::size_t result = 0;

  for (auto s : {S...})
    result += s;

  return result;
}

template < bool... >
struct bool_pack
{
};

template < bool... Bs >
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
struct sensor_base_tag
{
};

// A sensor needs to include ...
template < std::size_t MeasurementSize, std::size_t NumLinearStates = 0,
           std::size_t NumRotationStates = 0 >
struct sensor_base : sensor_base_tag
{
  static constexpr auto measurement_vector_size()
  {
    return std::integral_constant< std::size_t, MeasurementSize >{};
  }

  static constexpr auto num_linear_states()
  {
    return std::integral_constant< std::size_t, NumLinearStates >{};
  }

  static constexpr auto num_rotation_states()
  {
    return std::integral_constant< std::size_t, NumRotationStates >{};
  }
};

// msf2 specs, takes a list of unique sensors
template < typename... Sensors >
struct msf2_specs
{
  // Check that input list is contains only unique sensors
  static_assert(sizeof...(Sensors) > 0,
                "There are no sensors defined and there must be at least one.");

  // Check that input list is actually sensors
  static_assert(
      all_true< std::is_base_of< sensor_base_tag, Sensors >::value... >(),
      "Not all elements in the sensor list is a sensor, did you forget to "
      "inherit from sensor_base?");

  // Check that input list is contains only unique sensors
  static_assert(
      mpl::call< distinct<>, Sensors... >::value,
      "The list does not only contain unique sensors, remove duplicates.");

  // Define state sizes
  using num_core_states       = std::integral_constant< std::size_t, 16 >;
  using num_core_error_states = std::integral_constant< std::size_t, 15 >;
  using num_sensor_states     = std::integral_constant<
      std::size_t,
      sum< Sensors::num_linear_states()... >() +  // Get num linear state
          4 * sum< Sensors::num_rotation_states()... >()  // Get num rotational
                                                          // states
      >;
  using num_sensor_error_states = std::integral_constant<
      std::size_t,
      sum< Sensors::num_linear_states()... >() +  // Get num linear state
          3 * sum< Sensors::num_rotation_states()... >()  // Get num rotational
                                                          // states
      >;

  // Storage for sensors with state
  using sensor_storage_t = std::tuple< Sensors... >;
};

template < typename Spec >
class msf2
{
private:
  typename Spec::sensor_storage_t sensor_storage;

public:
  // Convenience functions to get parts of state vector
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

  template < typename T >
  constexpr std::size_t get()
  {
    static_assert(
        std::is_base_of< sensor_base_tag, T >::value,
        "Only sensors and state enums can be an argument in get< ... >().");
    static_assert(T::num_linear_states() > 0,
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
    static_assert(T::num_rotation_states() > 0,
                  "Can't get sensor state, this sensor has no extra rotation "
                  "states defined.");

    return 10001;
  }
};

// Make some sensors
// Fake sensors, just to make it happy
struct sensor1 : sensor_base< 1,   // Measurement size (1)
                              0,   // Number of extra linear states
                              0 >  // Number of extra rotational states (1 state
                                   // = 1 extra quaternion)
{
};

struct sensor2 : sensor_base< 3, 1, 0 >
{
};

struct sensor3 : sensor_base< 1, 1, 1 >
{
};

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
