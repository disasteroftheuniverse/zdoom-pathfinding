class JSON {
	
	//char literals
	
	const TAB = 0x9;			// '\t'
	const LF = 0xA;				// '\n'
	const CR = 0xD;				// '\r'
	const SPACE = 0x20;			// ' '
	const NUM_0 = 0x30;			// '0'
	const NUM_9 = 0x39;			// '9'
	const _DOT = 0x2E;			// '.'
	const COMMA = 0x2C;			// ','
	const COLON = 0x3A;			// ':'
	const QUOTE_1 = 0x27;		// '\''
	const QUOTE_2 = 0x22;		// '\"'
	const BACKSLASH = 0x5C;		// '\\'
	const SLASH = 0x2F;			// '/'
	const ASTERISK = 0x2A;		// '*'
	const SQUARE_OPEN = 0x5B;	// '['
	const SQUARE_CLOSE = 0x5D;	// ']'
	const CURLY_OPEN = 0x7B;	// '{'
	const CURLY_CLOSE = 0x7D;	// '}'
	const PLUS = 0x2B;			// '+'
	const MINUS = 0x2D;			// '-'
	
	private static bool isWhitespace(int c){
		return c==TAB||c==LF||c==CR||c==SPACE;
	}
	
	private static bool isNumber(int c){
		return c>=NUM_0&&c<=NUM_9;
	}
	
	private static int getEscape(int c){//DOESN'T SUPPORT UNICODE/HEX/OCTAL
		switch(c){
		case 0x61://a
			return 0x07;
		case 0x62://b
			return 0x08;
		case 0x65://e
			return 0x1B;
		case 0x6E://n
			return 0x0A;
		case 0x72://r
			return 0x0D;
		case 0x74://t
			return 0x09;
		case 0x76://v
			return 0x0B;
		default:
			return c;
		}
	}
	
	private static int needsEscape(int c,bool single_quote){
		switch(c){
		case 0x07://a
		case 0x08://b
		case 0x1B://e
		case 0x0A://n
		case 0x0D://r
		case 0x09://t
		case 0x0B://v
			return true;
		case QUOTE_1:
			return single_quote;
		case QUOTE_2:
			return !single_quote;
		default:
			return false;
		}
	}
	
	private static int makeEscape(int c){
		switch(c){
		case 0x07://a
			return 0x61;
		case 0x08://b
			return 0x62;
		case 0x1B://e
			return 0x65;
		case 0x0A://n
			return 0x6E;
		case 0x0D://r
			return 0x72;
		case 0x09://t
			return 0x74;
		case 0x0B://v
			return 0x76;
		default:
			return c;
		}
	}
	
    //skip whitespace and comments
	private static void skipWhitespace(out string data,out uint i,uint len,out uint line){
		if(i>=len)return;
		//while data[i] is whitespace, cr/lf or tab, advance index
		for(uint c,ii;i<len;){
			[c,ii]=data.getNextCodePoint(i);
			if(!isWhitespace(c)){
				if(ii<len&&c==SLASH){
					uint i3;
					[c,i3]=data.getNextCodePoint(ii);
					if(c==SLASH){
						//if is single line comment, skip until next LF or EOF
						ii=i3;
						for(;ii<len;){
							[c,ii]=data.getNextCodePoint(ii);
							if(c==LF){
								line++;
								break;
							}
						}
					}else if(c==ASTERISK){
						//if is multiline comment, skip until next '*/'
						ii=i3;
						for(;ii<len;){
							[c,ii]=data.getNextCodePoint(ii);
							if(c==ASTERISK&&ii<len){
								[c,ii]=data.getNextCodePoint(ii);
								if(c==SLASH){
									break;
								}
							}
						}
					}else{
						break;
					}
				}else{
					break;
				}
			}else if(c==LF){
				line++;
			}
			i=ii;
		}
	}
	
    //parse a string
	private static JsonElementOrError parseString(out string data,out uint i,uint len){
		if(i>=len) return JsonError.make("Expected String, got EOF");
		uint delim,ii;
		[delim,ii]=data.getNextCodePoint(i);
		if(delim!=QUOTE_1&&delim!=QUOTE_2){
			return JsonError.make("Expected  ''' or '\"' (String), got "..data.mid(i,1));
		}
		i=ii;
		JsonString s=JsonString.make();
		uint c,i3;
		for(;ii<len;){
			[c,i3]=data.getNextCodePoint(ii);
			if(c==delim){
				s.s.appendFormat("%s",data.mid(i,ii-i));
				i=i3;
				return s;
			}
			if(c==BACKSLASH){
				if(i3>=len){
					return JsonError.make("On String, expected Character, got EOF");
				}
				s.s.appendFormat("%s",data.mid(i,ii-i));
				[c,ii]=data.getNextCodePoint(i3);
				s.s.appendCharacter(getEscape(c));
				i=ii;
			}else if(c==LF){
				return JsonError.make("On String, expected Character, got EOL");
			}else{
				ii=i3;
			}
		}
		string delim_s="";
		delim_s.appendCharacter(delim);
		return JsonError.make("On String, expected '"..delim_s.."', got EOF");
	}
	
    //parse a json object, allows trailing commas
	private static JsonElementOrError parseObject(out string data,out uint i,uint len,out uint line) {
		if(i>=len) return JsonError.make("Expected Object, got EOF");
		uint c,ii;
		[c,ii]=data.getNextCodePoint(i);
		if(c!=CURLY_OPEN){
			return JsonError.make("Expected '{' (Object), got '"..data.mid(i,1).."'");
		}
		i=ii;
		JsonObject obj=JsonObject.make();
        string last_element;
        bool has_last_element=false;
		for(;i<len;){
			skipWhitespace(data,i,len,line);
			[c,ii]=data.getNextCodePoint(i);
			if(c==CURLY_CLOSE){
				i=ii;
				return obj;
			}
			let key=parseString(data,i,len);
			if(key is "JsonError"){
                if(has_last_element){
                    return JsonError.make("After Object value '"..last_element.."', "..JsonError(key).what);
                }else{
                    return JsonError.make("On first Object value, "..JsonError(key).what);
                }
			}
            last_element=JsonString(key).s;
            has_last_element=true;
			skipWhitespace(data,i,len,line);
			if(i>=len){
				return JsonError.make("On Object value '"..last_element.."', expected ':', got EOF");
			}
			[c,ii]=data.getNextCodePoint(i);
			if(c!=COLON){
				return JsonError.make("On Object value '"..last_element.."', expected ':', got '"..data.mid(i,1).."'");
			}
			i=ii;
			skipWhitespace(data,i,len,line);
			if(i>=len){
				return JsonError.make("On Object value '"..last_element.."', expected element, got EOF");
			}
			let elem=parseElement(data,i,len,line);
			if(elem is "JsonError"){
				return JsonError.make("On Object value '"..last_element.."', "..JsonError(elem).what);
			}
			obj.set(JsonString(key).s,JsonElement(elem));
			skipWhitespace(data,i,len,line);
			if(i>=len){
				return JsonError.make("After Object value '"..last_element.."', expected ',', got EOF after element '"..last_element.."'");
			}
			[c,ii]=data.getNextCodePoint(i);
			if(c!=COMMA){
				if(c==CURLY_CLOSE){
					continue;
				}
				return JsonError.make("After Object value '"..last_element.."', expected ',', got '"..data.mid(i,1).."'");
			}
			i=ii;
		}
        if(has_last_element){
            return JsonError.make("After Object value '"..last_element.."', expected }, got EOF");
        }else{
            return JsonError.make("On Empty Object, expected }, got EOF");
        }
	}
	
    //parse a json array, allows trailing commas
	private static JsonElementOrError parseArray(out string data,out uint i,uint len,out uint line) {
		if(i>=len) return JsonError.make("Expected Array, got EOF");
		uint c,ii;
		[c,ii]=data.getNextCodePoint(i);
		if(c!=SQUARE_OPEN){
			return JsonError.make("Expected '[' (Array), got '"..data.mid(i,1).."'");
		}
		i=ii;
		JsonArray arr=JsonArray.make();
		for(;i<len;){
			skipWhitespace(data,i,len,line);
			[c,ii]=data.getNextCodePoint(i);
			if(c==SQUARE_CLOSE){
				i=ii;
				return arr;
			}
			let elem=parseElement(data,i,len,line);
			if(elem is "JsonError"){
				return JsonError.make("On Array index "..arr.size()..", "..JsonError(elem).what);
			}
			arr.push(JsonElement(elem));
			skipWhitespace(data,i,len,line);
			if(i>=len){
				return JsonError.make("On Array index "..(arr.size()-1)..", expected ',', got EOF");
			}
			[c,ii]=data.getNextCodePoint(i);
			if(c!=COMMA){
				if(c==SQUARE_CLOSE){
					continue;
				}
				return JsonError.make("After Array index "..(arr.size()-1)..", expected ',', got '"..data.mid(i,1).."'");
			}
			i=ii;
		}
        if(arr.size()==0){
            return JsonError.make("On Empty Array, expected ], got EOF");
        }else{
            return JsonError.make("After Array index "..(arr.size()-1)..", expected ], got EOF");
            
        }
	}
	
    //parse a number in the format [0-9]+(?:\.[0-9]+)?
	private static JsonElementOrError parseNumber(out string data,out uint i,uint len) {
		if(i>=len) return JsonError.make("Expected Number, got EOF");
		uint ii,i3,c;
		[c,ii]=data.getNextCodePoint(i);
		if(!isNumber(c)) return JsonError.make("Expected '0'-'9' (Number), got '"..data.mid(i,1).."'");
		ii=i;
		bool is_double=false;
		for(;ii<data.length();){
			[c,i3]=data.getNextCodePoint(ii);
			if(c==_DOT){
				if(is_double){
					return JsonError.make("On Number, duplicate dot");
				}
				is_double=true;
			}else if(!isNumber(c)){
				break;
			}
			ii=i3;
		}
		uint n=ii-i;
		JsonElement o;
		if(is_double){
			o=JsonDouble.make(data.mid(i,n).toDouble());
		}else{
			o=JsonInt.make(data.mid(i,n).toInt());
		}
		i=ii;
		return o;
	}
	
	//returns one of: JsonArray, JsonObject, JsonString, JsonInt, JsonDouble, JsonNull, JsonError
	private static JsonElementOrError parseElement(out string data,out uint i,uint len,out uint line){
		skipWhitespace(data,i,len,line);
		if(i>=len){
			return JsonError.make("Expected JSON Element, got EOF");
		}
		uint c,ii;
		[c,ii]=data.getNextCodePoint(i);
		if(isNumber(c)){//number
			return parseNumber(data,i,len);
		}else if(c==PLUS||c==MINUS){
			i=ii;
			skipWhitespace(data,i,len,line);
			let num=parseNumber(data,i,len);
			if(c==MINUS && num is "JsonNumber"){
				return JsonNumber(num).negate();
			}else{
				return num;
			}
		}else if(c==SQUARE_OPEN){//array
			return parseArray(data,i,len,line);
		}else if(c==CURLY_OPEN){//object
			return parseObject(data,i,len,line);
		}else if(c==QUOTE_1||c==QUOTE_2){//string
			return parseString(data,i,len);
		}else if(data.mid(i,4)=="true"){//bool, true
			i+=4;
			return JsonBool.make(true);
		}else if(data.mid(i,5)=="false"){//bool, false
			i+=5;
			return JsonBool.make(false);
		}else if(data.mid(i,4)=="null"){//null
			i+=4;
			return JsonNull.make();
		}else{
			return JsonError.make("Expected JSON Element, got '"..data.mid(i,1).."'");
		}
	}
	
	// roughly O(n), has extra complexity from data structures (DynArray, HashTable) and string copying
	static JsonElementOrError parse(string json_string,bool allow_data_past_end=false){
		uint index=0;
		uint line=1;
		uint len=json_string.length();
		JsonElementOrError elem=parseElement(json_string,index,len,line);
		if(!(elem is "JsonError")){
			skipWhitespace(json_string,index,len,line);
			if(index<len&&!allow_data_past_end&&!((index==(len-1)&&json_string.getNextCodePoint(index)==0))){
				return JsonError.make("On JSON line "..line.." - expected EOF, got '"..json_string.mid(index,1).."'");
			}
		}else{
			return JsonError.make("On JSON line "..line.." - "..JsonError(elem).what);
		}
		return elem;
	}
	
	static string serialize_string(string s){
		String o;
		uint len=s.length();
		o.AppendCharacter(QUOTE_2);
		uint i=0,c;
		while(i<len){
			[c,i]=s.getNextCodePoint(i);
			if(needsEscape(c,false)){
				o.AppendCharacter(BACKSLASH);
				o.AppendCharacter(makeEscape(c));
			}else{
				o.AppendCharacter(c);
			}
		}
		o.AppendCharacter(QUOTE_2);
		return o;
	}
}
