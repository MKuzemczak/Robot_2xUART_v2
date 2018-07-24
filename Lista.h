#pragma once

#include <list>

template<typename T>
	class Lista : public std::list<T>
	{
	public:
		T & operator [](const int index)
		{
			int i = 0;
			for (T & r : (*this))
			{
				if (i == index)
					return r;
				i++;
			}
		}

		void erase(int index)
		{
			int i = 0;

			for (auto it = std::list<T>::begin(); it != std::list<T>::end();)
			{
				if (i++ == index)
				{
					it = std::list<T>::erase(it);
					break;
				}
				else
					++it;
			}

		}
		
		auto iteratorAt(int i)
		{
			int j = 0;
			for (auto it = (*this).begin(); j <= i; it++)
			{
				if (j == i)
					return it;
				
				j++;
			}
		}
	};