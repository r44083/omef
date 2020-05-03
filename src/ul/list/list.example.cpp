#include "ul/list/list.hpp"

int main(void)
{
	ul::list<int> mylist;
	
	ul::list_elem<int> *elem1 = new ul::list_elem<int>(5);
	ul::list_elem<int> *elem2 = new ul::list_elem<int>(6);
	
	mylist.add(elem1);
	mylist.add(elem2);
	
	ul::list_elem<int> *finded = mylist.find(5);
	mylist.remove(finded);
	delete finded;
	
	mylist.~list();
}