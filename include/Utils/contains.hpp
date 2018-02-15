#include <algorithm>

// reference: <https://codereview.stackexchange.com/questions/59997/contains-algorithm-for-stdvector>

template<class C, class T>
auto contains(const C& vector, const T& entry)
	-> decltype(end(vector), true)
{
	    return end(vector) != std::find(begin(vector), end(vector), entry);
}
