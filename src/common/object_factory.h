/*
 * Copyright (C) 2021, Ajay Pal S. Grewal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see https://opensource.org/licenses/GPL-2.0
 */
#ifndef WIFLX_COMMON_OBJECT_FACTORY_H
#define WIFLX_COMMON_OBJECT_FACTORY_H

#include <unordered_map>
#include <string>

#define WIFLX_OBJECT_FACTORY_REGISTER_IMPL(base, impl)  \
	bool impl ## _registered = base::get_factory ().register_impl<impl> (#impl)

#define WIFLX_OBJECT_FACTORY_REGISTER_IMPL_WITH_NAME(base, impl, name)  \
	bool impl ## _registered = base::get_factory ().register_impl<impl> (name)

namespace wiflx {
namespace common {

/**
 * This class is used to create objects of concrete subclasses that derive
 * from a polymorpic base class. The polymorphgic base class should store
 * a static instance of this object factory.
 */
template <typename BASE, typename ...Args>
class object_factory
{
public:
	typedef BASE* (*constructor) (Args&& ...args);

	/**
	 * Construct object factory.
	 */
	object_factory ()
	{}

	/**
	 * Destroys this object.
	 */
	~object_factory ()
	{}

	/*
	 * \brief Register an implementation subclass of a polyporphic base class.
	 *
	 * \param name The name of the subclass.
	*/
	template <typename IMPL>
	bool register_impl (const std::string &name)
	{
		struct maker
		{
			static BASE* create ( Args&& ...args )
			{
				return (new IMPL (std::forward<Args>(args)... ));
			}
		};

		auto result = m_impl_map.insert (std::make_pair (name, &maker::create));
		if (!result.second)
		{
			throw std::runtime_error ("duplicate implementation");
		}

		return true;
	}

	/*
	 * \brief Create an instance of the subclass of a polyporphic base class.
	 *
	 * \param name The name of the subclass  registered with the factory.
	*/
	BASE* create_impl (const std::string &name, Args&& ...args)
	{
		const auto it = m_impl_map.find (name);
		if (it == m_impl_map.end ())
		{
			return nullptr;
		}

		return it->second (std::forward<Args>(args)...);
	}

private:
	std::unordered_map<std::string, constructor> m_impl_map;
};

} // namespace common
} // namespace wiflx

#endif // WIFLX_COMMON_OBJECT_FACTORY_H
