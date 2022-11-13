class JsonElementOrError {
}

class JsonElement : JsonElementOrError abstract {
	abstract string serialize();
}

class JsonNumber : JsonElement abstract {
	abstract JsonNumber negate();
}

class JsonInt : JsonNumber {
	int i;
	static JsonInt make(int i=0){
		JsonInt ii=new("JsonInt");
		ii.i=i;
		return ii;
	}
	override JsonNumber negate(){
		i=-i;
		return self;
	}
	override string serialize(){
		return ""..i;
	}
}

class JsonDouble : JsonNumber {
	double d;
	static JsonDouble make(double d=0){
		JsonDouble dd=new("JsonDouble");
		dd.d=d;
		return dd;
	}
	override JsonNumber negate(){
		d=-d;
		return self;
	}
	override string serialize(){
		return ""..d;
	}
}

class JsonBool : JsonElement {
	bool b;
	static JsonBool make(bool b=false){
		JsonBool bb=new("JsonBool");
		bb.b=b;
		return bb;
	}
	override string serialize(){
		return b?"true":"false";
	}
}

class JsonString : JsonElement {
	string s;
	static JsonString make(string s=""){
		JsonString ss=new("JsonString");
		ss.s=s;
		return ss;
	}
	override string serialize(){
		return JSON.serialize_string(s);
	}
}

class JsonNull : JsonElement {
	static JsonNull make(){
		return new("JsonNull");
	}
	override string serialize(){
		return "null";
	}
}

class JsonError : JsonElementOrError {
	String what;
	static JsonError make(string s){
		JsonError e=new("JsonError");
		e.what=s;
		return e;
	}
}