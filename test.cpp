#include <cstdint>
#include <type_traits>
#include <initializer_list>
#include <tuple>
#include <Eigen/Dense>

#include <kvasir/mpl/mpl.hpp>

// Helper functions
namespace mpl = kvasir::mpl;

template < typename Comp = mpl::is_same<>, typename IsSame = mpl::is_same<>,
           typename C = mpl::identity >
using distinct = mpl::fork< mpl::size<>,               // Fork 1: size of input
                            mpl::stable_sort<          // Fork 2: size of unique list
                                Comp,                  // Comparison for sorting the list
                                mpl::remove_adjacent<  // Remove non unique items
                                    IsSame,            // Comparison for uniqueness
                                    mpl::size<>        // Get the size
                                    > >,
                            mpl::is_same<  // Check if the sizes are the same
                                C          // Continuation
                                > >;

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
    return std::is_same< bool_pack< true, Bs... >, bool_pack< Bs..., true > >::value;
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
}  // namespace states

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

    static constexpr auto num_quaternion_states()
    {
        return std::integral_constant< std::size_t, NumRotationStates >{};
    }
};

// msf2 specs, takes a list of unique sensors
template < typename... Sensors >
struct msf2_sensors
{
    // Check that input list is existing
    static_assert(sizeof...(Sensors) > 0,
                  "There are no sensors defined and there must be at least one.");

    // Check that input list is actually sensors
    static_assert(all_true< std::is_base_of< sensor_base_tag, Sensors >::value... >(),
                  "Not all elements in the sensor list is a sensor, did you forget to "
                  "inherit from sensor_base?");

    // Check that input list is contains only unique sensors
    static_assert(mpl::call< distinct<>, Sensors... >::value,
                  "The list does not only contain unique sensors, remove duplicates.");

    // Define core state sizes
    static constexpr std::size_t num_sensor_states =
        sum< Sensors::num_linear_states()... >() +         // Get num linear state
        4 * sum< Sensors::num_quaternion_states()... >();  // Get num
                                                           // rotational
                                                           // states

    // Define error state sizes
    static constexpr std::size_t num_sensor_error_states =
        sum< Sensors::num_linear_states()... >() +         // Get num linear state
        3 * sum< Sensors::num_quaternion_states()... >();  // Get num
                                                           // rotational
                                                           // states

    // Storage for sensors with state
    using sensor_storage = std::tuple< Sensors... >;
};

template < typename... Sensors >
class msf2
{
private:
    using sensors_t = msf2_sensors< Sensors... >;

    static constexpr std::size_t num_core_states = 16;
    static constexpr std::size_t num_core_error_states = 15;
    static constexpr std::size_t num_states = num_core_states + sensors_t::num_sensor_states;
    static constexpr std::size_t num_error_states =
        num_core_error_states + sensors_t::num_sensor_error_states;

    using state_t = Eigen::Matrix< double, num_states, 1 >;
    using error_state_t = Eigen::Matrix< double, num_error_states, 1 >;
    using covariance = Eigen::Matrix< double, num_error_states, num_error_states >;

    state_t state;
    error_state_t error_state;
    typename sensors_t::sensor_storage sensor_storage;

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

    template < class Sensor >
    constexpr Sensor& get_sensor()
    {
        return std::get<Sensor>(sensor_storage);
    }

    template < typename T >
    constexpr std::size_t get_linear()
    {
        static_assert(std::is_base_of< sensor_base_tag, T >::value,
                      "Only sensors can be an argument in get_linear< ... >().");
        static_assert(T::num_linear_states() > 0,
                      "Can't get sensor state, this sensor has no extra linear "
                      "states defined.");

        return 9001;
    }

    template < typename T >
    constexpr std::size_t get_attitude()
    {
        static_assert(std::is_base_of< sensor_base_tag, T >::value,
                      "Only sensors can be an argument in get_rotation< ... >().");
        static_assert(T::num_quaternion_states() > 0,
                      "Can't get sensor state, this sensor has no extra quaternion "
                      "states defined.");

        return 10001;
    }
};

#include <iostream>

// Make some sensors
// Fake sensors, just to make it happy
struct sensor1 : sensor_base< 1,   // Measurement size (1)
                              0,   // Number of extra linear states
                              0 >  // Number of extra rotational states (1 state
                                   // = 1 extra quaternion)
{
    void hello()
    {
        std::cout << "Hello from sensor1\n";
    }
};

struct sensor2 : sensor_base< 3, 1, 0 >
{
    void hello()
    {
        std::cout << "Hello from sensor2\n";
    }
};

struct sensor3 : sensor_base< 1, 1, 1 >
{
    void hello()
    {
        std::cout << "Hello from sensor3\n";
    }
};

// Create an MSF specification from it
// Create the MSF2 filter
using msf = msf2< sensor1, sensor2, sensor3 >;

int main()
{
    msf my_msf;

    // Try get stuff
    std::cout << "val : " << my_msf.get_attitude< sensor3 >() << "\n";
    std::cout << "val2: " << sizeof(sensor3) << "\n";
    std::cout << "val3: " << sizeof(msf) / 8 << "\n";
    my_msf.get_sensor<sensor3>().hello() ;

    return 0;
}
