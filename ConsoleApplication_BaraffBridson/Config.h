#ifndef CONFIG_H
#define CONFIG_H

#include <string>

class Config
{
public:
	static std::string const modelPath;
	static std::string const spherePath;

private:
	Config() {}

};

#endif
