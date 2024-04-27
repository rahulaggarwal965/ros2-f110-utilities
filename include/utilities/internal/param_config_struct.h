#pragma once

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

#include "<pfr/core.hpp>"
#include "<pfr/core_name.hpp"

namespace params {

namespace internal {

template<typename T> concept Parameter = requires(T t) { std::cout << t; };
template <typename T> concept ParameterVector = requires {
    typename std::vector<typename T::value_type>;
    requires Parameter<typename T::value_type>;
};

template<Parameter T>
void print_param(std::string param_name, T &param) {
    std::string upper_param_name{param_name};
    std::transform(upper_param_name.begin(), upper_param_name.end(), upper_param_name.begin(), ::toupper);
    std::cout << upper_param_name << ": " << param << std::endl;
}

template<Parameter T>
void print_param(std::string param_name, const std::vector<T> &param) {
    std::string upper_param_name{param_name};
    std::transform(upper_param_name.begin(), upper_param_name.end(), upper_param_name.begin(), ::toupper);
    std::cout << upper_param_name << ": [";
    for (auto i = 0; i < param.size() - 1; i++) {
       std::cout << param[i] << ", ";
    }
    std::cout << param.back() << "]" << std::endl;
}

template<typename T>
T traverse_config(T &config, const std::string &prefix = "") {
    constexpr auto param_names = pfr::names_as_array<T>();
    pfr::for_each_field(
        config,
        [&prefix, &param_names]<typename P>(P &param, size_t i) {
            std::string full_param_name = prefix + std::string{param_names[i]};
            if constexpr(Parameter<P> || ParameterVector<P>) {
                print_param(full_param_name, param);
            } else {
                param = traverse_config(param, full_param_name + ".");
            }
        }
    );
    return config;
}

template<typename T>
T register_config(T &config) {
    config = traverse_config(config);

    return config;
}

/* void update_config(); */
/* template<typename T> */
/* rcl_interfaces::msg::SetParametersResult update_config(T& config, const std::vector<rclcpp::Parameter> &parameters) { */
/*     constexpr auto param_names = pfr::names_as_array<T>(); */
/*     for (const auto &param_update : parameters) { */
/*         pfr::for_each_field( */
/*             config, */
/*             [&param_names, param_update](auto &param, size_t i) { */
/*                 if (param_names[i] == param_update.get_name()) { */
/*                     param = param_update.get_value<decltype(param)>(); */
/*                 } */
/*             } */
/*         ); */
/*     } */
/**/
/*     rcl_interfaces::msg::SetParametersResult result; */
/*     result.successful = true; */
/*     result.reason = "success"; */
/*     return result; */
/* } */

}

}

