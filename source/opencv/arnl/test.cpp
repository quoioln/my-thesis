int main() {

	ifstream is("positions.txt");

	string	 line = "";
	char[] t = new char[100];
	while (!is.eof()) {
		is >> t;
		//cout <<line[0]<<endl;
		cout <<t<<endl;
	}
	is.close();
	return 0;
}
