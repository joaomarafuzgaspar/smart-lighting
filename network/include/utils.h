#ifndef UTILS_H
#define UTILS_H

#include <string>

class RawString {
public:
    RawString(const std::string& str) : str {str} {}

    friend std::ostream& operator<<(std::ostream& os, const RawString& rs)
    {
        for (auto chr : rs.str) {
            switch (chr) {
                case '\n':
                    std::cout << "\\n";
                    break;
                case '\r':
                    std::cout << "\\r";
                    break;
                default:
                    std::cout << chr;
                    break;
            }
        }
        return os;
    }

private:
    std::string str;
};

bool starts_with(const std::string& str, const std::string& prefix)
{
    return str.substr(0, prefix.size()) == prefix;
}

#endif