/******************************************************************************/
/*!
\file		GlobalBlackboard.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Singleton-based global blackboard for behavior agents

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <any>
#include <unordered_map>
#include <mutex>
#include <type_traits>

class GlobalBlackboard
{
public:
    // Delete copy constructor and assignment operator to ensure singleton integrity
    GlobalBlackboard(const GlobalBlackboard&) = delete;
    GlobalBlackboard& operator=(const GlobalBlackboard&) = delete;

    // Get the singleton instance
    static GlobalBlackboard& get_instance();

    template <typename T>
    void set_value(const char* key, const T& value);

    template <typename T>
    T get_value(const char* key) const;

private:
    // Private constructor to prevent external instantiation
    GlobalBlackboard() = default;

    // Data storage
    std::unordered_map<const char*, std::any> data;

    // Mutex for thread-safety
    mutable std::mutex mutex;
};

inline GlobalBlackboard& GlobalBlackboard::get_instance()
{
    static GlobalBlackboard instance; // Thread-safe in C++11 and later
    return instance;
}

template<typename T>
inline void GlobalBlackboard::set_value(const char* key, const T& value)
{
    static_assert(std::is_copy_constructible<T>::value, "Attempting to add non-copy-constructible type into blackboard");

    std::lock_guard<std::mutex> lock(mutex); // Ensure thread safety
    data[key] = value;
}

template<typename T>
inline T GlobalBlackboard::get_value(const char* key) const
{
    std::lock_guard<std::mutex> lock(mutex); // Ensure thread safety

    const auto result = data.find(key);

    if (result != data.end())
    {
        try
        {
            return std::any_cast<T>(result->second);
        }
        catch (const std::bad_any_cast&)
        {
            __debugbreak(); // Debug if the type mismatch occurs
        }
    }
    else
    {
        __debugbreak(); // Debug if the key does not exist
    }

    return T(); // Return default-constructed value if something goes wrong
}
