using namespace std;

struct node{
	string question;
	string object;
	node * left;
	node * right;
};

class btree{
	public:
		btree();
		~btree();
		
		void insert(string question, string object);
		void destroy_tree();

	private:
		void destroy_tree(node * leaf);
		void insert(string question, string object, node * leaf);		

		node * root;
};

btree::btree(){
	root = NULL;
}

void btree::destroy_tree(node * leaf){
	if(leaf != NULL){
		destroy_tree(leaf->left);
		destroy_tree(leaf->right);
		delete leaf;
	}
}

