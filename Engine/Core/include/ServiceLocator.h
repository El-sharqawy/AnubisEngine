#pragma once

// Service Locator class written by Osama Elsharqawy 1/4/2025

#include "Singleton.h"
#include <unordered_map>
#include <ctype.h>
#include <typeindex>

class CServiceLocator : public CSingleton<CServiceLocator>
{
public:
	template <typename T>
	inline static void Register(T* service)
	{
		assert(service != nullptr && "Registering nullptr service");
		ms_mServices[std::type_index(typeid(T))] = service;
	}

	template <typename T>
	inline static T& Get()
	{
		T* pService = GetPtr<T>();
		assert(pService != nullptr && "Service not registered");
		return *pService;
	}

	template <typename T>
	inline static T* GetPtr()
	{
		auto it = ms_mServices.find(std::type_index(typeid(T)));
		if (it == ms_mServices.end())
			return nullptr;
		return static_cast<T*>(it->second);
	}

	template <typename T>
	static bool IsRegistered()
	{
		return ms_mServices.find(std::type_index(typeid(T))) != ms_mServices.end();
	}

	static void Clear()   // call on shutdown
	{
		ms_mServices.clear();
	}

private:
	inline static std::unordered_map<std::type_index, void*> ms_mServices;
};

