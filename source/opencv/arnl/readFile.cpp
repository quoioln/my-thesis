#include<iostream>
#include<fstream>
#include<Aria.h>
#define ARRAY_SIZE(array)(sizeof(array[0])/sizeof(array))
using namespace std;
/*
ArPose parsePose(string input){
	for (int i = 0; i < input.length(); i++) {
		ArPose pose = NULL;
		int index = input.find(" ");
//		if (strcmp("", input[i])) {

//		}
		if (index < 0) {
			cout << "sai vi tri";
		} else {
			atof(input.substr(0, index));
//			pose.setX(stof(input.substr(0, index)));
		}
	}

}
*/

//list<ArPose> readPostitions(char* fileName){
ArPose* readPostitions(char* fileName){
	ArPose* postitionList = new ArPose[1000];
//	list<ArPose> postitionList;
//	postitionList.
	ArPose pose;
//	ifstream is("positions.txt");
	ifstream is(fileName);
	char line[20];
	bool check = true;
	int i = 0;
	while (!is.eof()) {
		is >>line;
		cout <<"*"<<atoi(line)<<"*"<<endl;
		if (check) {
			pose.setX(atoi(line));
			check = false;
		} else {
			pose.setY(atoi(line));
			postitionList[i] = pose;
//			postitionList.push_back(pose);
			check = true;
			i++;
		}
	}
//	for ()
//	for (std::list<int>::iterator it = postitionList.begin(); it != postitionList.end(); it++)
//		(ArPose)*it.
//	    std::cout << it.<< ' ';

	is.close();
//	for (int i = 0 ; i < postitionList.size(); i++) {
//		cout <<"("<<postitionList. front().getX()<<", "<<postitionList.front().getY()<<")"<<endl;
//		postitionList.pop_back();

//	}
	int len = sizeof(postitionList[0])/sizeof(postitionList);
//	cout<<sizeof(postitionList[0]);
//	cout<<sizeof(postitionList);
	cout<<ARRAY_SIZE(postitionList);
	for (int i = 0 ; i < len; i++) {
			cout <<"("<<postitionList[i].getX()<<", "<<postitionList[i].getY()<<")"<<endl;
	}
	return postitionList;
}

int main() {
//	list<ArPose> listPose =
			readPostitions("positions.txt");
//	fo
/*
	ifstream is("positions.txt");Lo

//	int n;
//	is >> n;
	//string	 line = "";
	char line[100];
//	ArPose* poseList[] = new ArPose[n];
//	cin.ignore();
	bool check = false;
	while (!is.eof()) {
//		is >>line;
		cout <<atof(line)<<endl;
	}
	is.close();
	return 0;
	*/
}
