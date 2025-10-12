#if 0 // examples

// 01-basic.cpp

#define FLAGS_IMPLEMENTATION
#include "flags.hpp"
#include <vector>
#include <iostream>


int main(int argc, char** argv)
{
    Flags::parse(argc, argv)
        ->with_arg("test", 't', 100, "int test value")
        ->with_arg<std::string>("str", 's', "some str", "int test sstr")
        ->with_arg("args", 'a', std::vector<int>{1, 3, 4}, "some arg list")
        ->with_opt("option", 'o', "option test")
        ->set_help("basic example");

    std::cout << "test: " << Flags::arg<int>("test") << "\n";
    std::cout << "str: " << Flags::arg<std::string>("str") << "\n";
    std::cout << "option: " << Flags::opt("option") << "\n";

    std::cout << "args: ";
    auto args{Flags::arg<std::vector<int>>("args")};
    for (auto it : args) {
        std::cout << it << " ";
    }
    std::cout << "\n";

    std::cout << "Parameters: ";
    for (const auto& it : Flags::raw_params()) {
        std::cout << it << " ";
    }
    std::cout << "\n";

    return 0;
}

#endif // examples

#ifndef FLAGS_HPP_
#define FLAGS_HPP_

#include <unordered_map>
#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <any>

#define GLAGS_UNKNOWN_OPTION(NAME)                  \
    do {                                            \
        std::cerr << "Bad option: " << NAME;        \
        assert(false && "Unknown option");          \
    } while (0)

#define GLAGS_DUPLICATED_OPTION(NAME)               \
    do {                                            \
        std::cerr << "Refine optino: " << NAME;     \
        assert(false && "Bad option");              \
    } while (0)

#define GLAGS_CHECK_OPTION_DUPLICATED(NAME)         \
    do {                                            \
        if (data_.find(NAME) != data_.end()) {      \
            GLAGS_DUPLICATED_OPTION(NAME);          \
        }                                           \
    } while (0)

template<typename>
struct flags_is_vector : std::false_type {};

template<typename T, typename A>
struct flags_is_vector<std::vector<T, A>> : std::true_type {};

template<typename T>
T flags_cast_string(const std::string& v)
{
    assert(false && "Not supported string cast");
}

template<>
inline int flags_cast_string<int>(const std::string& v)
{
    return std::stoi(v);
}

template<>
inline float flags_cast_string<float>(const std::string& v)
{
    return std::stof(v);
}

template<>
inline double flags_cast_string<double>(const std::string& v)
{
    return std::stod(v);
}

template<>
inline long flags_cast_string<long>(const std::string& v)
{
    return std::stol(v);
}

template<>
inline long long flags_cast_string<long long>(const std::string& v)
{
    return std::stoll(v);
}

template<>
inline unsigned long flags_cast_string<unsigned long>(const std::string& v)
{
    return std::stoul(v);
}

template<>
inline unsigned long long flags_cast_string<unsigned long long>(const std::string& v)
{
    return std::stoull(v);
}

template<>
inline std::string flags_cast_string<std::string>(const std::string& v)
{
    return v;
}

bool flags_is_short_option(char* arg);
bool flags_is_long_option(char* arg);
bool flags_is_option(char* arg);

struct Flags
{
    template<typename T>
    Flags* with_arg(const std::string& name, char shortcut, T value, const std::string& desc);
    template<typename T>
    Flags* with_arg(const std::string& name, char shortcut, const std::vector<T>& value, const std::string& desc);
    Flags* with_opt(const std::string& name, char shortcut, const std::string& desc);
    void set_help(const std::string& desc);

    static Flags* parse(int argc, char** argv);
    template<typename T>
    typename std::enable_if<!flags_is_vector<T>::value, T>::type
    static arg(const std::string& key);
    template<typename T>
    typename std::enable_if<flags_is_vector<T>::value, T>::type
    static arg(const std::string& key);
    template<typename T>
    static std::vector<T> params();
    static const std::vector<std::string>& raw_params();
    static bool opt(const std::string& key);
private:
    template<typename T>
    struct Arg {
        std::string long_name;
        T value;
        bool has_extra_arg;
        char shor_name;
    }; // struct Arg

    struct ArgParser
    {
        ArgParser(int argc, char** argv);
        bool find_option_long_name(const std::string& name);
        bool find_option_short_name(char shortcut);
        void gather_extra(std::vector<std::string>& extra_args);
        template<typename T>
        bool expect_param(T& value);
        template<typename T>
        bool expect_params(std::vector<T>& value);

        char** argv;
        int argc;
        int shortcut_index;
        int token_index;
    }; // struct ArgParser

    std::unordered_map<std::string, std::any> data_;
    std::vector<std::string> args_;
    std::string app_name_;
    std::stringstream help_doc_ss_;
    std::vector<std::string> input_args_;
    bool exit_with_help_;
    int argc_;
    char** argv_;

    static Flags* instance();
    Flags() : exit_with_help_(false) {}
    void show_help(const std::string& desc);
}; // struct Flags

template<typename T>
Flags* Flags::with_arg(const std::string& name, char shortcut, T value, const std::string& desc)
{
    GLAGS_CHECK_OPTION_DUPLICATED(name);

    help_doc_ss_ << "\n  -" << shortcut << ", --" << name << " <" << value << ">" << "\t\t"<< desc;
    Arg<T> arg;
    arg.long_name = name;
    arg.shor_name = shortcut;
    arg.value = value;
    arg.has_extra_arg = true;

    ArgParser ap(argc_, argv_);
    if (ap.find_option_long_name(name) || ap.find_option_short_name(shortcut)) {
        if (!ap.expect_param<T>(arg.value)) {
            exit_with_help_ = true;
        }
    }
    ap.gather_extra(args_);

    data_.insert(std::make_pair(name, arg));
    return this;
}

template<typename T>
Flags* Flags::with_arg(const std::string& name, char shortcut, const std::vector<T>& value, const std::string& desc)
{
    GLAGS_CHECK_OPTION_DUPLICATED(name);

    help_doc_ss_ << "\n  -" << shortcut << ", --" << name << " <";
    for (size_t i = 0; i < value.size(); ++i) {
        help_doc_ss_ << value.at(i);
        if (i < value.size() - 1) help_doc_ss_ << ", ";
    }    
    help_doc_ss_ << ">\t\t"<< desc;

    Arg<std::vector<T>> arg;
    arg.long_name = name;
    arg.shor_name = shortcut;
    arg.value = value;
    arg.has_extra_arg = true;

    ArgParser ap(argc_, argv_);
    if (ap.find_option_long_name(name) || ap.find_option_short_name(shortcut)) {
        arg.value.clear();
        if (!ap.expect_params<T>(arg.value)) {
            exit_with_help_ = true;
        }
    }
    ap.gather_extra(args_);

    data_.insert(std::make_pair(name, arg));
    return this;
}

template<typename T>
typename std::enable_if<!flags_is_vector<T>::value, T>::type
Flags::arg(const std::string& key)
{
    auto ins{instance()};
    auto find_result{ins->data_.find(key)};
    if (find_result == ins->data_.end()) {
        GLAGS_UNKNOWN_OPTION(key);
    }

    return std::any_cast<Arg<T>>(find_result->second).value;
}

template<typename T>
typename std::enable_if<flags_is_vector<T>::value, T>::type
Flags::arg(const std::string& key)
{
    auto ins{instance()};
    auto find_result{ins->data_.find(key)};
    if (find_result == ins->data_.end()) {
        GLAGS_UNKNOWN_OPTION(key);
    }

    return std::any_cast<Arg<T>>(find_result->second).value;
}

template<typename T>
std::vector<T> Flags::params()
{
    auto ins{instance()};
    std::vector<T> result;
    for (const auto& it : ins->args_) {
        result.push_back(flags_cast_string<T>(it));
    }
    return result;
}

template<typename T>
bool Flags::ArgParser::expect_param(T& value)
{
    int next{token_index + 1};
    if (next >= argc) {
        token_index = next;
        return false;
    }

    if (shortcut_index > 0 && (shortcut_index + 1) != strlen(argv[token_index])) {
        return false;
    }

    if (flags_is_option(argv[next])) {
        token_index = next;
        return false;
    }

    token_index = next;
    value = flags_cast_string<T>(argv[next]);
    return true;
}

template<typename T>
bool Flags::ArgParser::expect_params(std::vector<T>& value)
{
    int next{token_index + 1};
    if (next >= argc) {
        return false;
    }

    if (shortcut_index > 0 && (shortcut_index + 1) != strlen(argv[token_index])) {
        return false;
    }

    value.clear();
    while (next < argc && !flags_is_option(argv[next])) {
        value.push_back(flags_cast_string<T>(argv[next]));
        ++next;
    }

    token_index = next - 1;
    return !value.empty();
}

#endif // FLAGS_HPP_


// ============= IMPLEMENTATION ============

// #define FLAGS_IMPLEMENTATION

#ifdef FLAGS_IMPLEMENTATION

#include <mutex>

void Flags::show_help(const std::string& desc)
{
    std::cout << app_name_ << ": "
        << desc << "\nOptions:"
        << help_doc_ss_.str() << "\n\n";
    exit(0);
}

void Flags::set_help(const std::string& desc)
{
    const std::string name{"help"};
    const char shortcut{'h'};

    ArgParser ap(argc_, argv_);
    if (ap.find_option_long_name(name) || ap.find_option_short_name(shortcut)) {
        exit_with_help_ = true;
    }

    if (exit_with_help_) {
        show_help(desc);
    }
}

bool flags_is_option(char* arg)
{
    if (arg && arg[0] == '-') return true;
    return false;
}

bool flags_is_short_option(char* arg)
{
    if (arg && strlen(arg) >= 2) {
        if (arg[0] != '-') {
            return false;
        }

        if (arg[1] != '-') {
            return true;
        }
    }
    return false;
}

bool flags_is_long_option(char* arg)
{
    if (arg && strlen(arg) >= 3) {
        if (arg[0] == '-' && arg[1] == '-') {
            if (arg[2] == '-') {
                return false;
            }
            return true;
        }
    }
    return false;
}

Flags* Flags::instance()
{
    static Flags *instance_{nullptr};
    static std::once_flag flag;
    if (!instance_) {
        std::call_once(flag, [&]() -> void {
            instance_ = new (std::nothrow) Flags();
        });
    }
    return instance_; 
}

Flags* Flags::parse(int argc, char** argv)
{
    auto ins{instance()};
    ins->argc_ = argc;
    ins->argv_ = argv;
    ins->app_name_ = argv[0];
    for (int i = 1; i < argc; ++i) {
        ins->input_args_.push_back(argv[i]);
    }

    ArgParser ap(argc, argv);
    ap.gather_extra(ins->args_);
    return ins;
}

Flags* Flags::with_opt(const std::string& name, char shortcut, const std::string& desc)
{
    GLAGS_CHECK_OPTION_DUPLICATED(name);

    help_doc_ss_ << "\n  -" << shortcut << ", --" << name << "\t\t"<< desc;
    Arg<bool> arg;
    arg.long_name = name;
    arg.shor_name = shortcut;
    arg.has_extra_arg = false;
    arg.value = false;

    ArgParser ap(argc_, argv_);
    if (ap.find_option_long_name(name) || ap.find_option_short_name(shortcut)) {
        arg.value = true;
    }
    ap.gather_extra(args_);

    data_.insert(std::make_pair(name, arg));

    return this;
}

bool Flags::opt(const std::string& key)
{
    auto ins{instance()};
    auto find_result{ins->data_.find(key)};
    if (find_result == ins->data_.end()) {
        GLAGS_UNKNOWN_OPTION(key);
    }

    return std::any_cast<Arg<bool>>(find_result->second).value;
}

const std::vector<std::string>& Flags::raw_params()
{
    auto ins{instance()};
    return ins->args_;
}

Flags::ArgParser::ArgParser(int argc, char** argv)
    : argv(argv)
    , argc(argc)
    , shortcut_index(-1)
    , token_index(0)
{}

bool Flags::ArgParser::find_option_long_name(const std::string& name)
{
    for (int i = 1; i < argc; ++i) {
        if (flags_is_long_option(argv[i])) {
            if (strcmp((argv[i] + 2), name.c_str()) == 0) {
                token_index = i;
                return true;
            }
        }
    }
    return false;
}

bool Flags::ArgParser::find_option_short_name(char shortcut)
{

    for (int i = 1; i < argc; ++i) {
        if (flags_is_short_option(argv[i])) {
            auto this_arg{argv[i]};
            auto opt_len{strlen(this_arg)};
            for (int j = 1; j < opt_len; ++j) {
                if (this_arg[j] == shortcut) {
                    token_index = i;
                    shortcut_index = j;
                    return true;
                }
            }
        }
    }
    return false;
}

void Flags::ArgParser::gather_extra(std::vector<std::string>& extra_args)
{
    int next{token_index + 1};
    if (next >= argc) {
        return;
    }

    while (next < argc && !flags_is_option(argv[next])) {
        extra_args.push_back(argv[next]);
        ++next;
    }
}

#endif // FLAGS_IMPLEMENTATION
