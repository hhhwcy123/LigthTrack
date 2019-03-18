#ifndef ARGUMENTPARSER_H
#define ARGUMENTPARSER_H

#include <map>
#include <string>
#include <ostream>


double asciiToDouble(const char* str);
/** Convert a ascii number to a float, ignoring locale settings.*/
inline float asciiToFloat(const char* str) { return static_cast<float>(asciiToDouble(str)); }


class  ApplicationUsage 
{
public:

	template<typename T>
	inline T maximum(T lhs, T rhs) { return lhs>rhs ? lhs : rhs; }
	template<typename T>
	inline T minimum(T lhs, T rhs) { return lhs<rhs ? lhs : rhs; }

	static ApplicationUsage* instance();

	ApplicationUsage() {}

	ApplicationUsage(const std::string& commandLineUsage);

	typedef std::map<std::string, std::string> UsageMap;


	/** The ApplicationName is often displayed when logging errors, and frequently incorporated into the Description (below). */
	void setApplicationName(const std::string& name) { _applicationName = name; }
	const std::string& getApplicationName() const { return _applicationName; }

	/** If non-empty, the Description is typically shown by the Help Handler
	* as text on the Help display (which also lists keyboard abbreviations. */
	void setDescription(const std::string& desc) { _description = desc; }
	const std::string& getDescription() const { return _description; }

	enum Type
	{
		NO_HELP = 0x0,
		COMMAND_LINE_OPTION = 0x1,
		ENVIRONMENTAL_VARIABLE = 0x2,
		KEYBOARD_MOUSE_BINDING = 0x4,
		HELP_ALL = KEYBOARD_MOUSE_BINDING | ENVIRONMENTAL_VARIABLE | COMMAND_LINE_OPTION
	};

	void addUsageExplanation(Type type, const std::string& option, const std::string& explanation);

	void setCommandLineUsage(const std::string& explanation) { _commandLineUsage = explanation; }
	const std::string& getCommandLineUsage() const { return _commandLineUsage; }


	void addCommandLineOption(const std::string& option, const std::string& explanation, const std::string &defaultValue = "");

	void setCommandLineOptions(const UsageMap& usageMap) { _commandLineOptions = usageMap; }
	const UsageMap& getCommandLineOptions() const { return _commandLineOptions; }

	void setCommandLineOptionsDefaults(const UsageMap& usageMap) { _commandLineOptionsDefaults = usageMap; }
	const UsageMap& getCommandLineOptionsDefaults() const { return _commandLineOptionsDefaults; }


	void addEnvironmentalVariable(const std::string& option, const std::string& explanation, const std::string& defaultValue = "");

	void setEnvironmentalVariables(const UsageMap& usageMap) { _environmentalVariables = usageMap; }
	const UsageMap& getEnvironmentalVariables() const { return _environmentalVariables; }

	void setEnvironmentalVariablesDefaults(const UsageMap& usageMap) { _environmentalVariablesDefaults = usageMap; }
	const UsageMap& getEnvironmentalVariablesDefaults() const { return _environmentalVariablesDefaults; }


	void addKeyboardMouseBinding(const std::string& prefix, int key, const std::string& explanation);
	void addKeyboardMouseBinding(int key, const std::string& explanation);
	void addKeyboardMouseBinding(const std::string& option, const std::string& explanation);

	void setKeyboardMouseBindings(const UsageMap& usageMap) { _keyboardMouse = usageMap; }
	const UsageMap& getKeyboardMouseBindings() const { return _keyboardMouse; }


	void getFormattedString(std::string& str, const UsageMap& um, unsigned int widthOfOutput = 80, bool showDefaults = false, const UsageMap& ud = UsageMap());

	void write(std::ostream& output, const UsageMap& um, unsigned int widthOfOutput = 80, bool showDefaults = false, const UsageMap& ud = UsageMap());

	void write(std::ostream& output, unsigned int type = COMMAND_LINE_OPTION, unsigned int widthOfOutput = 80, bool showDefaults = false);

	void writeEnvironmentSettings(std::ostream& output);

protected:

	virtual ~ApplicationUsage() { }

	std::string _applicationName;
	std::string _description;
	std::string _commandLineUsage;
	UsageMap    _commandLineOptions;
	UsageMap    _environmentalVariables;
	UsageMap    _keyboardMouse;
	UsageMap    _environmentalVariablesDefaults;
	UsageMap    _commandLineOptionsDefaults;

};

class  ArgumentParser
{
public:

	class  Parameter
	{
	public:
		enum ParameterType
		{
			BOOL_PARAMETER,
			FLOAT_PARAMETER,
			DOUBLE_PARAMETER,
			INT_PARAMETER,
			UNSIGNED_INT_PARAMETER,
			STRING_PARAMETER
		};

		union ValueUnion
		{
			bool*           _bool;
			float*          _float;
			double*         _double;
			int*            _int;
			unsigned int*   _uint;
			std::string*    _string;
		};

		Parameter(bool& value) { _type = BOOL_PARAMETER; _value._bool = &value; }

		Parameter(float& value) { _type = FLOAT_PARAMETER; _value._float = &value; }

		Parameter(double& value) { _type = DOUBLE_PARAMETER; _value._double = &value; }

		Parameter(int& value) { _type = INT_PARAMETER; _value._int = &value; }

		Parameter(unsigned int& value) { _type = UNSIGNED_INT_PARAMETER; _value._uint = &value; }

		Parameter(std::string& value) { _type = STRING_PARAMETER; _value._string = &value; }

		Parameter(const Parameter& param) { _type = param._type; _value = param._value; }

		Parameter& operator = (const Parameter& param) { _type = param._type; _value = param._value; return *this; }

	

		bool valid(const char* str) const;
		bool assign(const char* str);

	protected:

		ParameterType   _type;
		ValueUnion      _value;
	};

	/** Return true if the specified string is an option in the form
	* -option or --option. */
	static bool isOption(const char* str);

	/** Return true if string is non-NULL and not an option in the form
	* -option or --option. */
	static bool isString(const char* str);

	/** Return true if specified parameter is a number. */
	static bool isNumber(const char* str);

	/** Return true if specified parameter is a bool. */
	static bool isBool(const char* str);

public:

	ArgumentParser(int* argc, char **argv);

	void setApplicationUsage(ApplicationUsage* usage) { _usage = usage; }
	ApplicationUsage* getApplicationUsage() { return _usage; }
	const ApplicationUsage* getApplicationUsage() const { return _usage; }

	/** Return the argument count. */
	int& argc() { return *_argc; }

	/** Return the argument array. */
	char** argv() { return _argv; }

	/** Return the char* argument at the specified position. */
	char* operator [] (int pos) { return _argv[pos]; }

	/** Return the const char* argument at the specified position. */
	const char* operator [] (int pos) const { return _argv[pos]; }

	/** Return the application name, as specified by argv[0]  */
	std::string getApplicationName() const;

	/** Return the position of an occurrence of a string in the argument list.
	* Return -1 if no string is found. */
	int find(const std::string& str) const;

	int find(const int startPos, const int endPos, const std::string& str) const;
	/** Return true if the specified parameter is an option in the form of
	* -option or --option. */
	bool isOption(int pos) const;

	/** Return true if the specified parameter is a string not in
	* the form of an option. */
	bool isString(int pos) const;

	/** Return true if the specified parameter is a number. */
	bool isNumber(int pos) const;

	bool containsOptions() const;

	/** Remove one or more arguments from the argv argument list,
	* and decrement the argc respectively. */
	void remove(int pos, int num = 1);

	/** Return true if the specified argument matches the given string. */
	bool match(int pos, const std::string& str) const;

	/** Search for an occurrence of a string in the argument list. If found,
	* remove that occurrence and return true. Otherwise, return false. */
	bool read(const std::string& str);
	bool read(const std::string& str, Parameter value1);
	bool read(const std::string& str, Parameter value1, Parameter value2);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7);
	bool read(const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7, Parameter value8);


	/** If the argument value at the specified position matches the given string,
	* and subsequent parameters are also matched, then set the parameter values,
	* remove the arguments from the list, and return true. Otherwise, return false. */
	bool read(int pos, const std::string& str);
	bool read(int pos, const std::string& str, Parameter value1);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7);
	bool read(int pos, const std::string& str, Parameter value1, Parameter value2, Parameter value3, Parameter value4, Parameter value5, Parameter value6, Parameter value7, Parameter value8);


	enum ErrorSeverity
	{
		BENIGN = 0,
		CRITICAL = 1
	};

	typedef std::map<std::string, ErrorSeverity> ErrorMessageMap;

	/** Return the error flag, true if an error has occurred when reading arguments. */
	bool errors(ErrorSeverity severity = BENIGN) const;

	/** Report an error message by adding to the ErrorMessageMap. */
	void reportError(const std::string& message, ErrorSeverity severity = CRITICAL);

	/** For each remaining option, report it as unrecognized. */
	void reportRemainingOptionsAsUnrecognized(ErrorSeverity severity = BENIGN);

	/** Return the error message, if any has occurred. */
	ErrorMessageMap& getErrorMessageMap() { return _errorMessageMap; }

	/** Return the error message, if any has occurred. */
	const ErrorMessageMap& getErrorMessageMap() const { return _errorMessageMap; }

	/** Write error messages to the given ostream, if at or above the given severity. */
	void writeErrorMessages(std::ostream& output, ErrorSeverity sevrity = BENIGN);


	/** This convenience method handles help requests on the command line.
	* Return the type(s) of help requested. The return value of this
	* function is suitable for passing into getApplicationUsage()->write().
	* If ApplicationUsage::NO_HELP is returned then no help commandline option
	* was found on the command line. */
	ApplicationUsage::Type readHelpType();


protected:

	int*                            _argc;
	char**                          _argv;
	ErrorMessageMap                 _errorMessageMap;
	ApplicationUsage*               _usage;
	


};

#endif //!ARGUMENTPARSER_H
