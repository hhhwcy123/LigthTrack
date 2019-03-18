#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <ostream>

#include "ArgumentParser.h"
#include "SimOptions.h"



class  Commandline
{
public:
	friend class SimOptions;
	
	Commandline() { _simOptions = new SimOptions(); }
	~Commandline() { delete _simOptions; };

	SimOptions* getSimOption() { return _simOptions; };
	void getUsage(ApplicationUsage& usage);
	int read(std::ostream& fout, ArgumentParser& arguments);

private:

	SimOptions* _simOptions;


};

#endif //!COMMANDLINE_H





