// Singleton class written by Osama Elsharqawy 1/4/2025

#pragma once

#include <cassert>
#include <cstdint>

template <typename T> class CSingleton
{
public:
	CSingleton()
	{
		// A singleton class must only be constructed once.
		// If this assert fails, it means a second instance was created.
		assert(ms_pSingleton == nullptr);

        // "this" here is a CSingleton<T>* because we are inside the base class.
        // But we actually want to store the pointer as T* (the derived type).
        //
        // In simple single inheritance, CSingleton<T>* and T* often point to
        // the same address. But with multiple inheritance, the CSingleton<T>
        // subobject may live at an offset inside T.
        //
        // So we compute the byte offset between:
        //   - a fake T* at address 1
        //   - a fake CSingleton<T>* at address 1
        //
        // Using address 1 is just an old trick to measure pointer adjustment
        // without a real object. The difference gives the offset needed to
        // convert a CSingleton<T>* into the correct T*.
		std::uintptr_t offset = (std::uintptr_t)(T*)1 - (std::uintptr_t)(CSingleton<T>*)1;

        // "this" is the current CSingleton<T>* subobject.
        // Add the computed offset so the pointer now points to the full T object,
        // then store it as the global singleton instance.
        ms_pSingleton = (T*)((std::uintptr_t)this + offset);
	}

    virtual ~CSingleton()
    {
        assert(ms_pSingleton);
        ms_pSingleton = nullptr;
    }

    static T& Instance()
    {
        assert(ms_pSingleton);
        return (*ms_pSingleton);
    }

    static T* InstancePtr()
    {
        assert(ms_pSingleton);
        return (ms_pSingleton);
    }

private:
	static T* ms_pSingleton;
};

template <typename T> T* CSingleton <T>::ms_pSingleton = nullptr;