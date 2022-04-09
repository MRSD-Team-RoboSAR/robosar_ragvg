#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>

#pragma region ParseIniFile

using namespace std;

/*
* \brief Generic configuration Class
*
*/
class Config {
	// Data  
protected:
	string m_Delimiter;  //!< separator between key and value  
	string m_Comment;    //!< separator between value and comments  
	std::map<string, string> m_Contents;  //!< extracted keys and values  

	typedef std::map<string, string>::iterator mapi;
	typedef std::map<string, string>::const_iterator mapci;
	// Methods  
public:

	Config(string filename, string delimiter = "=", string comment = "#");
	Config();
	template<class T> T Read(const string& in_key) const;  //!<Search for key and read value or optional default value, call as read<T>  
	template<class T> T Read(const string& in_key, const T& in_value) const;
	template<class T> bool ReadInto(T& out_var, const string& in_key) const;
	template<class T>
	bool ReadInto(T& out_var, const string& in_key, const T& in_value) const;
	bool FileExist(string filename);
	void ReadFile(string filename, string delimiter = "=", string comment = "#");

	// Check whether key exists in configuration  
	bool KeyExists(const string& in_key) const;

	// Modify keys and values  
	template<class T> void Add(const string& in_key, const T& in_value);
	void Remove(const string& in_key);

	// Check or change configuration syntax  
	string GetDelimiter() const { return m_Delimiter; }
	string GetComment() const { return m_Comment; }
	string SetDelimiter(const string& in_s)
	{
		string old = m_Delimiter;  m_Delimiter = in_s;  return old;
	}
	string SetComment(const string& in_s)
	{
		string old = m_Comment;  m_Comment = in_s;  return old;
	}

	// Write or read configuration  
	friend std::ostream& operator<<(std::ostream& os, const Config& cf);
	friend std::istream& operator >> (std::istream& is, Config& cf);

protected:
	template<class T> static string T_as_string(const T& t);
	template<class T> static T string_as_T(const string& s);
	static void Trim(string& inout_s);


	// Exception types  
public:
	struct File_not_found {
		string filename;
		File_not_found(const string& filename_ = string())
			: filename(filename_) {}
	};
	struct Key_not_found {  // thrown only by T read(key) variant of read()  
		string key;
		Key_not_found(const string& key_ = string())
			: key(key_) {}
	};
};

/* static */
template<class T>
string Config::T_as_string(const T& t)
{
	// Convert from a T to a string  
	// Type T must support << operator  
	std::ostringstream ost;
	ost << t;
	return ost.str();
}


/* static */
template<class T>
T Config::string_as_T(const string& s)
{
	// Convert from a string to a T  
	// Type T must support >> operator  
	T t;
	std::istringstream ist(s);
	ist >> t;
	return t;
}


/* static */
template<>
inline string Config::string_as_T<string>(const string& s)
{
	// Convert from a string to a string  
	// In other words, do nothing  
	return s;
}


/* static */
template<>
inline bool Config::string_as_T<bool>(const string& s)
{
	// Convert from a string to a bool  
	// Interpret "false", "F", "no", "n", "0" as false  
	// Interpret "true", "T", "yes", "y", "1", "-1", or anything else as true  
	bool b = true;
	string sup = s;
	for (string::iterator p = sup.begin(); p != sup.end(); ++p)
		*p = toupper(*p);  // make string all caps  
	if (sup == string("FALSE") || sup == string("F") ||
		sup == string("NO") || sup == string("N") ||
		sup == string("0") || sup == string("NONE"))
		b = false;
	return b;
}


template<class T>
T Config::Read(const string& key) const
{
	// Read the value corresponding to key  
	mapci p = m_Contents.find(key);
	if (p == m_Contents.end()) throw Key_not_found(key);
	return string_as_T<T>(p->second);
}


template<class T>
T Config::Read(const string& key, const T& value) const
{
	// Return the value corresponding to key or given default value  
	// if key is not found  
	mapci p = m_Contents.find(key);
	if (p == m_Contents.end()) return value;
	return string_as_T<T>(p->second);
}


template<class T>
bool Config::ReadInto(T& var, const string& key) const
{
	// Get the value corresponding to key and store in var  
	// Return true if key is found  
	// Otherwise leave var untouched  
	mapci p = m_Contents.find(key);
	bool found = (p != m_Contents.end());
	if (found) var = string_as_T<T>(p->second);
	return found;
}


template<class T>
bool Config::ReadInto(T& var, const string& key, const T& value) const
{
	// Get the value corresponding to key and store in var  
	// Return true if key is found  
	// Otherwise set var to given default  
	mapci p = m_Contents.find(key);
	bool found = (p != m_Contents.end());
	if (found)
		var = string_as_T<T>(p->second);
	else
		var = value;
	return found;
}


template<class T>
void Config::Add(const string& in_key, const T& value)
{
	// Add a key with given value  
	string v = T_as_string(value);
	string key = in_key;
	Trim(key);
	Trim(v);
	m_Contents[key] = v;
	return;
}

Config::Config(string filename, string delimiter,
	string comment)
	: m_Delimiter(delimiter), m_Comment(comment)
{
	// Construct a Config, getting keys and values from given file  

	std::ifstream in(filename.c_str());

	if (!in) throw File_not_found(filename);

	in >> (*this);
}


Config::Config()
	: m_Delimiter(string(1, '=')), m_Comment(string(1, '#'))
{
	// Construct a Config without a file; empty  
}



bool Config::KeyExists(const string& key) const
{
	// Indicate whether key is found  
	mapci p = m_Contents.find(key);
	return (p != m_Contents.end());
}


/* static */
void Config::Trim(string& inout_s)
{
	// Remove leading and trailing whitespace  
	static const char whitespace[] = " \n\t\v\r\f";
	inout_s.erase(0, inout_s.find_first_not_of(whitespace));
	inout_s.erase(inout_s.find_last_not_of(whitespace) + 1U);
}


std::ostream& operator<<(std::ostream& os, const Config& cf)
{
	// Save a Config to os  
	for (Config::mapci p = cf.m_Contents.begin();
		p != cf.m_Contents.end();
		++p)
	{
		os << p->first << " " << cf.m_Delimiter << " ";
		os << p->second << std::endl;
	}
	return os;
}

void Config::Remove(const string& key)
{
	// Remove key and its value  
	m_Contents.erase(m_Contents.find(key));
	return;
}

std::istream& operator >> (std::istream& is, Config& cf)
{
	// Load a Config from is  
	// Read in keys and values, keeping internal whitespace  
	typedef string::size_type pos;
	const string& delim = cf.m_Delimiter;  // separator  
	const string& comm = cf.m_Comment;    // comment  
	const pos skip = delim.length();        // length of separator  

	string nextline = "";  // might need to read ahead to see where value ends  

	while (is || nextline.length() > 0)
	{
		// Read an entire line at a time  
		string line;
		if (nextline.length() > 0)
		{
			line = nextline;  // we read ahead; use it now  
			nextline = "";
		}
		else
		{
			std::getline(is, line);
		}

		// Ignore comments  
		line = line.substr(0, line.find(comm));

		// Parse the line if it contains a delimiter  
		pos delimPos = line.find(delim);
		if (delimPos < string::npos)
		{
			// Extract the key  
			string key = line.substr(0, delimPos);
			line.replace(0, delimPos + skip, "");

			// See if value continues on the next line  
			// Stop at blank line, next line with a key, end of stream,  
			// or end of file sentry  
			bool terminate = false;
			while (!terminate && is)
			{
				std::getline(is, nextline);
				terminate = true;

				string nlcopy = nextline;
				Config::Trim(nlcopy);
				if (nlcopy == "") continue;

				nextline = nextline.substr(0, nextline.find(comm));
				if (nextline.find(delim) != string::npos)
					continue;

				nlcopy = nextline;
				Config::Trim(nlcopy);
				if (nlcopy != "") line += "\n";
				line += nextline;
				terminate = false;
			}

			// Store key and value  
			Config::Trim(key);
			Config::Trim(line);
			cf.m_Contents[key] = line;  // overwrites if key is repeated  
		}
	}

	return is;
}
bool Config::FileExist(string filename)
{
	bool exist = false;
	std::ifstream in(filename.c_str());
	if (in)
		exist = true;
	return exist;
}

void Config::ReadFile(string filename, string delimiter,
	string comment)
{
	m_Delimiter = delimiter;
	m_Comment = comment;
	std::ifstream in(filename.c_str());

	if (!in) throw File_not_found(filename);

	in >> (*this);
}

#pragma endregion ParseIniFIle