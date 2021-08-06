#include <common/gpio.h>

#include <fstream>
#include <common/log.h>

#define GPIO_ENABLE_FILE "/sys/kernel/debug/iio/iio:device1/adi,gpo-manual-mode-enable"
#define GPIO_SET_FILE "/sys/kernel/debug/iio/iio:device1/gpo_set"

namespace wiflx {
namespace common {

void gpio::init ()
{
	std::ofstream ofs (GPIO_ENABLE_FILE, std::ofstream::out);
	ofs << "1" << std::endl;
}

void gpio::set (const ID id)
{
	std::ofstream ofs (GPIO_SET_FILE, std::ofstream::out);
	ofs << id << " 1" << std::endl;
}

void gpio::clear (const ID id)
{
	std::ofstream ofs (GPIO_SET_FILE, std::ofstream::out);
	ofs << id << " 0" << std::endl;
}

} // namespace common
} // namespace wiflx
