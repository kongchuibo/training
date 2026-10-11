#include <iostream>//I don't have Chinese input method,so I can only use English
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {//The scope should include the boundary itself
            return true;
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() >= 10;//The length should not be less than ten
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');//Single quotation marks should be added outside the numble
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
    //int checked = 0;//NO use,so I delete it
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
//Test input:ping Wang JP1100998
//Expected output:INVALID