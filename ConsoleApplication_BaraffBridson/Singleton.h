#pragma once

template <typename T>
class Singleton
{
protected:
	Singleton() {}
	Singleton(const Singleton& other) = delete;
	Singleton(Singleton&& other) = delete;

	Singleton& operator = (const Singleton &t) = delete;
	Singleton& operator = (Singleton&& t) = delete;

	virtual ~Singleton() {}

public:
	static T * Instance()
	{
		if (!T::msInstance)
		{
			T::msInstance = new T();
		}

		return T::msInstance;
	}

protected:
	static T* msInstance;
};

template <typename T>
T* Singleton<T>::msInstance = nullptr;
