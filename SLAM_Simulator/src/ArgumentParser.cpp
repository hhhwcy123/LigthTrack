
#include <stdlib.h>
#include <string.h>
#include <set>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>

#include "ArgumentParser.h"

double asciiToDouble(const char* str)
{
	const char* ptr = str;

	// check if could be a hex number.
	if (strncmp(ptr, "0x", 2) == 0)
	{

		double value = 0.0;

		// skip over leading 0x, and then go through rest of string
		// checking to make sure all values are 0...9 or a..f.
		ptr += 2;
		while (
			*ptr != 0 &&
			((*ptr >= '0' && *ptr <= '9') ||
			(*ptr >= 'a' && *ptr <= 'f') ||
				(*ptr >= 'A' && *ptr <= 'F'))
			)
		{
			if (*ptr >= '0' && *ptr <= '9') value = value*16.0 + double(*ptr - '0');
			else if (*ptr >= 'a' && *ptr <= 'f') value = value*16.0 + double(*ptr - 'a' + 10);
			else if (*ptr >= 'A' && *ptr <= 'F') value = value*16.0 + double(*ptr - 'A' + 10);
			++ptr;
		}

		// OSG_NOTICE<<"Read "<<str<<" result = "<<value<<std::endl;
		return value;
	}

	ptr = str;

	bool    hadDecimal[2];
	double  value[2];
	double  sign[2];
	double  decimalMultiplier[2];

	hadDecimal[0] = hadDecimal[1] = false;
	sign[0] = sign[1] = 1.0;
	value[0] = value[1] = 0.0;
	decimalMultiplier[0] = decimalMultiplier[1] = 0.1;
	int pos = 0;

	// compute mantissa and exponent parts
	while (*ptr != 0 && pos<2)
	{
		if (*ptr == '+')
		{
			sign[pos] = 1.0;
		}
		else if (*ptr == '-')
		{
			sign[pos] = -1.0;
		}
		else if (*ptr >= '0' && *ptr <= '9')
		{
			if (!hadDecimal[pos])
			{
				value[pos] = value[pos] * 10.0 + double(*ptr - '0');
			}
			else
			{
				value[pos] = value[pos] + decimalMultiplier[pos] * double(*ptr - '0');
				decimalMultiplier[pos] *= 0.1;
			}
		}
		else if (*ptr == '.')
		{
			hadDecimal[pos] = true;
		}
		else if (*ptr == 'e' || *ptr == 'E')
		{
			if (pos == 1) break;

			pos = 1;
		}
		else
		{
			break;
		}
		++ptr;
	}

	if (pos == 0)
	{
		// OSG_NOTICE<<"Read "<<str<<" result = "<<value[0]*sign[0]<<std::endl;
		return value[0] * sign[0];
	}
	else
	{
		double mantissa = value[0] * sign[0];
		double exponent = value[1] * sign[1];
		//OSG_NOTICE<<"Read "<<str<<" mantissa = "<<mantissa<<" exponent="<<exponent<<" result = "<<mantissa*pow(10.0,exponent)<<std::endl;
		return mantissa*pow(10.0, exponent);
	}
}

ApplicationUsage::ApplicationUsage(const std::string& commandLineUsage) :
	_commandLineUsage(commandLineUsage)
{
}

ApplicationUsage* ApplicationUsage::instance()
{
	static ApplicationUsage* s_applicationUsage = new ApplicationUsage;
	return s_applicationUsage;
}


void ApplicationUsage::addUsageExplanation(Type type, const std::string& option, const std::string& explanation)
{
	switch (type)
	{
	case(COMMAND_LINE_OPTION):
		addCommandLineOption(option, explanation);
		break;
	case(ENVIRONMENTAL_VARIABLE):
		addEnvironmentalVariable(option, explanation);
		break;
	case(KEYBOARD_MOUSE_BINDING):
		//addKeyboardMouseBinding(option,explanation);
		break;
	default:
		break;
	}
}

void ApplicationUsage::addCommandLineOption(const std::string& option, const std::string& explanation, const std::string& defaultValue)
{
	_commandLineOptions[option] = explanation;
	_commandLineOptionsDefaults[option] = defaultValue;
}

void ApplicationUsage::addEnvironmentalVariable(const std::string& option, const std::string& explanation, const std::string& defaultValue)
{
	_environmentalVariables[option] = explanation;
	_environmentalVariablesDefaults[option] = defaultValue;
}

void ApplicationUsage::addKeyboardMouseBinding(const std::string& prefix, int key, const std::string& explanation)
{
	if (key != 0)
	{
		std::ostringstream ostr;
		ostr << prefix;

		if (key == ' ')
		{
			ostr << "Space";
		}
		else if (key != 0)
		{
			ostr << char(key);
		}

		_keyboardMouse[ostr.str()] = explanation;
	}
}


void ApplicationUsage::addKeyboardMouseBinding(int key, const std::string& explanation)
{
	addKeyboardMouseBinding("", key, explanation);
}

void ApplicationUsage::addKeyboardMouseBinding(const std::string& option, const std::string& explanation)
{
	_keyboardMouse[option] = explanation;
}

void ApplicationUsage::getFormattedString(std::string& str, const UsageMap& um, unsigned int widthOfOutput, bool showDefaults, const UsageMap& ud)
{

	unsigned int maxNumCharsInOptions = 0;
	ApplicationUsage::UsageMap::const_iterator citr;
	for (citr = um.begin();
		citr != um.end();
		++citr)
	{
		maxNumCharsInOptions = maximum(maxNumCharsInOptions, (unsigned int)citr->first.length());
	}


	unsigned int fullWidth = widthOfOutput;
	unsigned int optionPos = 2;
	unsigned int explanationPos = optionPos + maxNumCharsInOptions + 2;

	double ratioOfExplanationToOutputWidth = float(explanationPos) / float(widthOfOutput);
	double maxRatioOfExplanationToOutputWidth = 0.25f;

	if (ratioOfExplanationToOutputWidth > maxRatioOfExplanationToOutputWidth)
	{
		explanationPos = static_cast<unsigned int>(maxRatioOfExplanationToOutputWidth*float(widthOfOutput));
	}

	unsigned int defaultPos = 0;
	if (showDefaults)
	{
		defaultPos = explanationPos;
		explanationPos = optionPos + 8;
	}
	unsigned int explanationWidth = fullWidth - explanationPos;

	std::string line;

	for (citr = um.begin();
		citr != um.end();
		++citr)
	{
		line.assign(fullWidth, ' ');
		line.replace(optionPos, citr->first.length(), citr->first);
		unsigned int currentEndPos = optionPos + citr->first.length();

		if (showDefaults)
		{

			UsageMap::const_iterator ditr = ud.find(citr->first);
			if (ditr != ud.end())
			{
				if (currentEndPos + 1 >= defaultPos)
				{
					str += line; str += "\n";
					line.assign(fullWidth, ' ');
				}

				line.replace(defaultPos, std::string::npos, "");
				if (ditr->second != "")
				{
					line += "[";
					line += ditr->second;
					line += "]";
				}
				str += line;
				str += "\n";
				line.assign(fullWidth, ' ');

				currentEndPos = 0;
			}
		}

		const std::string& explanation = citr->second;
		std::string::size_type pos = 0;
		std::string::size_type offset = 0;
		bool firstInLine = true;
		if (!explanation.empty())
		{

			if (currentEndPos + 1>explanationPos)
			{
				str += line; str += "\n";
				line.assign(fullWidth, ' ');
			}

			while (pos<explanation.length())
			{
				if (firstInLine) offset = 0;

				// skip any leading white space.
				while (pos<explanation.length() && explanation[pos] == ' ')
				{
					if (firstInLine) ++offset;
					++pos;
				}

				firstInLine = false;

				std::string::size_type width = minimum((std::string::size_type)(explanation.length() - pos), (std::string::size_type)(explanationWidth - offset));
				std::string::size_type slashn_pos = explanation.find('\n', pos);

				unsigned int extraSkip = 0;
				bool concatinated = false;
				if (slashn_pos != std::string::npos)
				{
					if (slashn_pos<pos + width)
					{
						width = slashn_pos - pos;
						++extraSkip;
						firstInLine = true;
					}
					else if (slashn_pos == pos + width)
					{
						++extraSkip;
						firstInLine = true;
					}
				}

				if (pos + width<explanation.length())
				{
					// now reduce width until we get a space or a return
					// so that we ensure that whole words are printed.
					while (width>0 &&
						explanation[pos + width] != ' ' &&
						explanation[pos + width] != '\n') --width;

					if (width == 0)
					{
						// word must be longer than a whole line so will need
						// to concatenate it.
						width = explanationWidth - 1;
						concatinated = true;
					}
				}

				line.replace(explanationPos + offset, explanationWidth, explanation, pos, width);

				if (concatinated) { str += line; str += "-\n"; }
				else { str += line; str += "\n"; }

				// move to the next line of output.
				line.assign(fullWidth, ' ');

				pos += width + extraSkip;

			}
		}
		else
		{
			str += line; str += "\n";
		}
	}
}

void ApplicationUsage::write(std::ostream& output, const ApplicationUsage::UsageMap& um, unsigned int widthOfOutput, bool showDefaults, const ApplicationUsage::UsageMap& ud)
{
	std::string str;
	getFormattedString(str, um, widthOfOutput, showDefaults, ud);
	output << str << std::endl;
}

void ApplicationUsage::write(std::ostream& output, unsigned int type, unsigned int widthOfOutput, bool showDefaults)
{

	output << "Usage: " << getCommandLineUsage() << std::endl;
	bool needspace = false;
	if ((type&COMMAND_LINE_OPTION) && !getCommandLineOptions().empty())
	{
		output << "Options";
		if (showDefaults) output << " [and default value]";
		output << ":" << std::endl;
		write(output, getCommandLineOptions(), widthOfOutput, showDefaults, getCommandLineOptionsDefaults());
		needspace = true;
	}

	if ((type&ENVIRONMENTAL_VARIABLE) && !getEnvironmentalVariables().empty())
	{
		if (needspace) output << std::endl;
		output << "Environmental Variables";
		if (showDefaults) output << " [and default value]";
		output << ":" << std::endl;
		write(output, getEnvironmentalVariables(), widthOfOutput, showDefaults, getEnvironmentalVariablesDefaults());
		needspace = true;
	}

	if ((type&KEYBOARD_MOUSE_BINDING) && !getKeyboardMouseBindings().empty())
	{
		if (needspace) output << std::endl;
		output << "Keyboard and Mouse Bindings:" << std::endl;
		write(output, getKeyboardMouseBindings(), widthOfOutput);
		needspace = true;
	}

}


void ApplicationUsage::writeEnvironmentSettings(std::ostream& output)
{
	output << "Current Environment Settings:" << std::endl;

	unsigned int maxNumCharsInOptions = 0;
	ApplicationUsage::UsageMap::const_iterator citr;
	for (citr = getEnvironmentalVariables().begin();
		citr != getEnvironmentalVariables().end();
		++citr)
	{
		std::string::size_type len = citr->first.find_first_of("\n\r\t ");
		if (len == std::string::npos) len = citr->first.size();
		maxNumCharsInOptions = maximum(maxNumCharsInOptions, static_cast<unsigned int>(len));
	}

	unsigned int optionPos = 2;
	std::string line;

	for (citr = getEnvironmentalVariables().begin();
		citr != getEnvironmentalVariables().end();
		++citr)
	{
		line.assign(optionPos + maxNumCharsInOptions + 2, ' ');
		std::string::size_type len = citr->first.find_first_of("\n\r\t ");
		if (len == std::string::npos) len = citr->first.size();
		line.replace(optionPos, len, citr->first.substr(0, len));
		const char *cp = getenv(citr->first.substr(0, len).c_str());
		if (!cp) cp = "[not set]";
		else if (!*cp) cp = "[set]";
		line += std::string(cp) + "\n";

		output << line;
	}
	output << std::endl;
}


bool ArgumentParser::isOption(const char* str)
{
	return str && str[0] == '-';
}

bool ArgumentParser::isString(const char* str)
{
	if (!str) return false;
	return true;
	//return !isOption(str);
}

bool ArgumentParser::isBool(const char* str)
{
	if (!str) return false;

	return (strcmp(str, "True") == 0 || strcmp(str, "true") == 0 || strcmp(str, "TRUE") == 0 ||
		strcmp(str, "False") == 0 || strcmp(str, "false") == 0 || strcmp(str, "FALSE") == 0 ||
		strcmp(str, "0") == 0 || strcmp(str, "1") == 0);
}

bool ArgumentParser::isNumber(const char* str)
{
	if (!str) return false;

	bool hadPlusMinus = false;
	bool hadDecimalPlace = false;
	bool hadExponent = false;
	bool couldBeInt = true;
	bool couldBeFloat = true;
	int noZeroToNine = 0;

	const char* ptr = str;

	// check if could be a hex number.
	if (strncmp(ptr, "0x", 2) == 0)
	{
		// skip over leading 0x, and then go through rest of string
		// checking to make sure all values are 0...9 or a..f.
		ptr += 2;
		while (
			*ptr != 0 &&
			((*ptr >= '0' && *ptr <= '9') ||
			(*ptr >= 'a' && *ptr <= 'f') ||
				(*ptr >= 'A' && *ptr <= 'F'))
			)
		{
			++ptr;
		}

		// got to end of string without failure, therefore must be a hex integer.
		if (*ptr == 0) return true;
	}

	ptr = str;

	// check if a float or an int.
	while (*ptr != 0 && couldBeFloat)
	{
		if (*ptr == '+' || *ptr == '-')
		{
			if (hadPlusMinus)
			{
				couldBeInt = false;
				couldBeFloat = false;
			}
			else hadPlusMinus = true;
		}
		else if (*ptr >= '0' && *ptr <= '9')
		{
			noZeroToNine++;
		}
		else if (*ptr == '.')
		{
			if (hadDecimalPlace)
			{
				couldBeInt = false;
				couldBeFloat = false;
			}
			else
			{
				hadDecimalPlace = true;
				couldBeInt = false;
			}
		}
		else if (*ptr == 'e' || *ptr == 'E')
		{
			if (hadExponent || noZeroToNine == 0)
			{
				couldBeInt = false;
				couldBeFloat = false;
			}
			else
			{
				hadExponent = true;
				couldBeInt = false;
				hadDecimalPlace = false;
				hadPlusMinus = false;
				noZeroToNine = 0;
			}
		}
		else
		{
			couldBeInt = false;
			couldBeFloat = false;
		}
		++ptr;
	}

	if (couldBeInt && noZeroToNine>0) return true;
	if (couldBeFloat && noZeroToNine>0) return true;

	return false;

}

bool ArgumentParser::Parameter::valid(const char* str) const
{
	switch (_type)
	{
	case Parameter::BOOL_PARAMETER:         return isBool(str); break;
	case Parameter::FLOAT_PARAMETER:        return isNumber(str); break;
	case Parameter::DOUBLE_PARAMETER:       return isNumber(str); break;
	case Parameter::INT_PARAMETER:          return isNumber(str); break;
	case Parameter::UNSIGNED_INT_PARAMETER: return isNumber(str); break;
	case Parameter::STRING_PARAMETER:       return isString(str); break;
	}
	return false;
}

bool ArgumentParser::Parameter::assign(const char* str)
{
	if (valid(str))
	{
		switch (_type)
		{
		case Parameter::BOOL_PARAMETER:
		{
			*_value._bool = (strcmp(str, "True") == 0 || strcmp(str, "true") == 0 || strcmp(str, "TRUE") == 0);
			break;
		}
		case Parameter::FLOAT_PARAMETER:        *_value._float = asciiToFloat(str); break;
		case Parameter::DOUBLE_PARAMETER:       *_value._double = asciiToDouble(str); break;
		case Parameter::INT_PARAMETER:          *_value._int = atoi(str); break;
		case Parameter::UNSIGNED_INT_PARAMETER: *_value._uint = atoi(str); break;
		case Parameter::STRING_PARAMETER:       *_value._string = str; break;
		}
		return true;
	}
	else
	{
		return false;
	}
}



ArgumentParser::ArgumentParser(int* argc, char **argv) :
	_argc(argc),
	_argv(argv),
	_usage(ApplicationUsage::instance())
{
#ifdef __APPLE__
	//On OSX, any -psn arguments need to be removed because they will
	// confuse the application. -psn plus a concatenated argument are
	// passed by the finder to application bundles
	for (int pos = 1; pos<this->argc(); ++pos)
	{
		if (std::string(_argv[pos]).compare(0, 4, std::string("-psn")) == 0)
		{
			remove(pos, 1);
		}
	}
#endif

#ifdef WIN32
	// Remove linefeed from last argument if it exist
	char* lastline = argc == 0 ? 0 : _argv[*argc - 1];
	if (lastline)
	{
		int len = strlen(lastline);
		if (len>0 && lastline[len - 1] == '\n') lastline[len - 1] = '\0';
	}
#endif
}

std::string ArgumentParser::getApplicationName() const
{
	if (_argc && *_argc>0) return std::string(_argv[0]);
	return "";
}


bool ArgumentParser::isOption(int pos) const
{
	return pos<*_argc && isOption(_argv[pos]);
}

bool ArgumentParser::isString(int pos) const
{
	return pos < *_argc && isString(_argv[pos]);
}

bool ArgumentParser::isNumber(int pos) const
{
	return pos < *_argc && isNumber(_argv[pos]);
}


int ArgumentParser::find(const std::string& str) const
{
	for (int pos = 1; pos<*_argc; ++pos)
	{
		if (str == _argv[pos])
		{
			return pos;
		}
	}
	return -1;
}

int ArgumentParser::find(const int startPos, const int endPos, const std::string& str) const
{
	for (int pos = startPos; pos<=endPos; ++pos)
	{
		if (str == _argv[pos])
		{
			return pos;
		}
	}
	return -1;
}

bool ArgumentParser::match(int pos, const std::string& str) const
{
	return pos<*_argc && str == _argv[pos];
}


bool ArgumentParser::containsOptions() const
{
	for (int pos = 1; pos<*_argc; ++pos)
	{
		if (isOption(pos)) return true;
	}
	return false;
}


void ArgumentParser::remove(int pos, int num)
{
	if (num == 0) return;

	for (; pos + num<*_argc; ++pos)
	{
		_argv[pos] = _argv[pos + num];
	}
	for (; pos<*_argc; ++pos)
	{
		_argv[pos] = 0;
	}
	*_argc -= num;
}

bool ArgumentParser::read(const std::string& str)
{
	int pos = find(str);
	if (pos <= 0) return false;
	remove(pos);
	return true;
}

bool ArgumentParser::read(const std::string& str, Parameter value1)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3, value4);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3, value4, value5);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3, value4, value5, value6);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3, value4, value5, value6, value7);
}

bool ArgumentParser::read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7, Parameter value8)
{
	int pos = find(str);
	if (pos <= 0) return false;
	return read(pos, str, value1, value2, value3, value4, value5, value6, value7, value8);
}

/** if the argument value at the position pos matches specified string, and subsequent
* Parameters are also matched then set the Parameter values and remove the from the list of arguments.*/
bool ArgumentParser::read(int pos, const std::string& str)
{
	if (match(pos, str))
	{
		remove(pos, 1);
		return true;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1)
{
	if (match(pos, str))
	{
		if ((pos + 1)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]))
			{
				value1.assign(_argv[pos + 1]);
				remove(pos, 2);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2)
{
	if (match(pos, str))
	{
		if ((pos + 2)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				remove(pos, 3);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3)
{
	if (match(pos, str))
	{
		if ((pos + 3)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				remove(pos, 4);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4)
{
	if (match(pos, str))
	{
		if ((pos + 4)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]) &&
				value4.valid(_argv[pos + 4]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				value4.assign(_argv[pos + 4]);
				remove(pos, 5);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5)
{
	if (match(pos, str))
	{
		if ((pos + 5)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]) &&
				value4.valid(_argv[pos + 4]) &&
				value5.valid(_argv[pos + 5]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				value4.assign(_argv[pos + 4]);
				value5.assign(_argv[pos + 5]);
				remove(pos, 6);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6)
{
	if (match(pos, str))
	{
		if ((pos + 6)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]) &&
				value4.valid(_argv[pos + 4]) &&
				value5.valid(_argv[pos + 5]) &&
				value6.valid(_argv[pos + 6]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				value4.assign(_argv[pos + 4]);
				value5.assign(_argv[pos + 5]);
				value6.assign(_argv[pos + 6]);
				remove(pos, 7);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7)
{
	if (match(pos, str))
	{
		if ((pos + 7)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]) &&
				value4.valid(_argv[pos + 4]) &&
				value5.valid(_argv[pos + 5]) &&
				value6.valid(_argv[pos + 6]) &&
				value7.valid(_argv[pos + 7]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				value4.assign(_argv[pos + 4]);
				value5.assign(_argv[pos + 5]);
				value6.assign(_argv[pos + 6]);
				value7.assign(_argv[pos + 7]);
				remove(pos, 8);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7, Parameter value8)
{
	if (match(pos, str))
	{
		if ((pos + 8)<*_argc)
		{
			if (value1.valid(_argv[pos + 1]) &&
				value2.valid(_argv[pos + 2]) &&
				value3.valid(_argv[pos + 3]) &&
				value4.valid(_argv[pos + 4]) &&
				value5.valid(_argv[pos + 5]) &&
				value6.valid(_argv[pos + 6]) &&
				value7.valid(_argv[pos + 7]) &&
				value8.valid(_argv[pos + 8]))
			{
				value1.assign(_argv[pos + 1]);
				value2.assign(_argv[pos + 2]);
				value3.assign(_argv[pos + 3]);
				value4.assign(_argv[pos + 4]);
				value5.assign(_argv[pos + 5]);
				value6.assign(_argv[pos + 6]);
				value7.assign(_argv[pos + 7]);
				value8.assign(_argv[pos + 8]);
				remove(pos, 9);
				return true;
			}
			reportError("argument to `" + str + "` is not valid");
			return false;
		}
		reportError("argument to `" + str + "` is missing");
		return false;
	}
	return false;
}

bool ArgumentParser::errors(ErrorSeverity severity) const
{
	for (ErrorMessageMap::const_iterator itr = _errorMessageMap.begin();
		itr != _errorMessageMap.end();
		++itr)
	{
		if (itr->second >= severity) return true;
	}
	return false;
}

void ArgumentParser::reportError(const std::string& message, ErrorSeverity severity)
{
	_errorMessageMap[message] = severity;
}


void ArgumentParser::writeErrorMessages(std::ostream& output, ErrorSeverity severity)
{
	for (ErrorMessageMap::iterator itr = _errorMessageMap.begin();
		itr != _errorMessageMap.end();
		++itr)
	{
		if (itr->second >= severity)
		{
			output << getApplicationName() << ": " << itr->first << std::endl;
		}
	}
}

ApplicationUsage::Type ArgumentParser::readHelpType()
{
	getApplicationUsage()->addCommandLineOption("-h or --help", "Display command line parameters");
	getApplicationUsage()->addCommandLineOption("--help-env", "Display environmental variables available");
	getApplicationUsage()->addCommandLineOption("--help-keys", "Display keyboard & mouse bindings available");
	getApplicationUsage()->addCommandLineOption("--help-all", "Display all command line, env vars and keyboard & mouse bindings.");

	// if user request help write it out to cout.
	if (read("--help-all"))             return ApplicationUsage::HELP_ALL;
	if (read("-h") || read("--help"))   return ApplicationUsage::COMMAND_LINE_OPTION;
	if (read("--help-env"))             return ApplicationUsage::ENVIRONMENTAL_VARIABLE;
	if (read("--help-keys"))            return ApplicationUsage::KEYBOARD_MOUSE_BINDING;

	return ApplicationUsage::NO_HELP;
}
