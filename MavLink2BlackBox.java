package org.unirail;

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;


public class MavLink2BlackBox {
	
	
	static final String trans(String datatype) {
		switch (datatype)
		{
			case "uint8_t":
			case "uint8_t_mavlink_version":
				return "@A byte ";
			case "uint16_t":
				return "@A short ";
			case "uint32_t":
				return "@A int ";
			case "uint64_t":
				return "@A long ";
			case "int8_t":
				return "byte ";
			case "int16_t":
				return "short ";
			case "int32_t":
				return "int ";
			case "int64_t":
				return "long ";
			case "float":
				return "float ";
			case "char":
				return "char ";
			case "double":
				return "double ";
			default:
				return null;
		}
	}
	
	
	static String MAV_CMD = "";
	
	private static int msgs_count = 0;
	private static int msgs_part  = 0;
	
	static class MyHandler extends DefaultHandler {
		
		public void parse(File file) {
			try
			{
				SAXParserFactory saxParserFactory = SAXParserFactory.newInstance();
				SAXParser        saxParser        = saxParserFactory.newSAXParser();
				saxParser.parse(file, this);
				
			} catch (Exception e) {}
		}
		
		public String enums = "";
		public String packs = "";
		
		private String  ENUM               = "";
		private String  ENUM_INITED        = "";
		private int     ENUM_INITED_POS    = 0;
		private String  ENUM_NOTINITED     = "";
		private int     ENUM_NOTINITED_POS = 0;
		private String  ENTRY              = "";
		private String  FIELD              = "";
		private String  MSG                = "";
		private boolean is_PARAM           = false;
		private boolean is_INITED          = false;
		private boolean is_File            = false;
		private boolean is_MAV_CMD         = false;
		private boolean is_opt             = false;
		
		
		@Override public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException {
			super.startElement(uri, localName, qName, attributes);
			//System.out.println(attributes.getValue("name"));
			switch (qName)
			{
				case "include":
					is_File = true;
					break;
				case "field":
					String datatype = attributes.getValue("type");
					
					int pos = datatype.indexOf("[");
					if (0 < pos)
					{
						String len = datatype.substring(pos + 1).replace("]", "");
						if (datatype.contains("char"))
							datatype = "@A(" + len + ") String ";
						else
							datatype = "@D (" + len + ")" + trans(datatype.substring(0, pos)) + "[] ";
					}
					else
						datatype = trans(datatype);
					
					if (is_opt)
					{
						if (datatype.startsWith("@")) datatype = datatype.substring(0, 2) + "_" + datatype.substring(2);
						else datatype = "@I_ " + datatype;
					}
					
					String e = attributes.getValue("enum");
					
					FIELD = (e == null ? datatype : e) + " " + attributes.getValue("name") + ";";
					break;
				case "message":
					MSG = "@id(" + attributes.getValue("id") + ") class " + attributes.getValue("name") + "{";
					msgs_count--;
					if (0 < msgs_count && msgs_count % msgs_part == 0)
						packs += "@@@@@@@%%%%%@@@@@";
					break;
				case "enum":
					if (attributes.getValue("name").equals("MAV_CMD"))
					{
						
						is_MAV_CMD = true;
						
					}
					
					ENUM = "enum " + attributes.getValue("name") + "\n{ ";
					break;
				case "entry":
					String value = attributes.getValue("value");
					if (is_INITED = value != null)
						ENTRY = attributes.getValue("name") + " = " + attributes.getValue("value") + ", ";
					else
						ENTRY = attributes.getValue("name") + ", ";
					
					break;
				
				case "param":
					is_PARAM = true;
					description += "\n" + attributes.getValue("index");
					break;
				case "description":
					description = "";
					break;
			}
		}
		
		String  description = null;
		boolean breaked     = false;//string already breaked
		
		@Override public void characters(char[] chs, int start, int length) throws SAXException {
			
			super.characters(chs, start, length);
			for (; (chs[start] < 33) && 0 < length; start++, length--) ;
			if (length < 1)
			{
				return;
			}
			if (is_PARAM)
			{
				breaked = true;
				description += "\t" + new String(chs, start, length);
				return;
			}
			
			if (is_File)//include other xml descriptor
			{
				final MyHandler handler = new MyHandler();
				handler.parse(new File(path + File.separator + new String(chs, start, length)));
				enums += handler.enums;
				packs += handler.packs;
			}
			
			for (int i = start, end = start + length; i < end && !(breaked = chs[i] == '\n'); i++) ;
			if (breaked)
			{
				description += new String(chs, start, length).replace("  ", "").replace("\t", "");
				return;
			}
			
			final int max_width = 100;
			
			if (length < max_width)
			{
				description = new String(chs, start, length);
				return;
			}
			
			StringBuffer sb = new StringBuffer(length + 10);
			
			for (int i = start, w = 0, end = start + length - 1; ; i++, w++)
				if (i == end)
				{
					sb.append(chs, i - w, w);
					break;
				}
				else
					if (chs[i] == ' ' && max_width < w)
					{
						breaked = true;
						chs[i] = '\n';
						sb.append(chs, i - w, w);
						w = 0;
						if (end - i < max_width)
						{
							sb.append(chs, i, end - i);
							break;
						}
					}
			
			description = sb.length() == 0 ? new String(chs, start, length) : sb.toString();
		}
		
		@Override public void endElement(String uri, String localName, String qName) throws SAXException {
			super.endElement(uri, localName, qName);
			switch (qName)
			{
				case "extensions":
					is_opt = true;
					break;
				case "include":
					is_File = false;
					break;
				case "enum":
					if (is_MAV_CMD)
					{
						MAV_CMD += ENUM_INITED;
						ENUM_INITED = "";
						is_MAV_CMD = false;
						
					}
					else
					{
						if (ENUM_NOTINITED != "")
						{
							int pos = ENUM_NOTINITED.length() - ENUM_NOTINITED_POS;
							enums += ENUM_NOTINITED.substring(0, pos) + "; " + ENUM_NOTINITED.substring(pos + 1);
							ENUM_NOTINITED = "";
						}
						else
							enums += "\n;\n";
						
						if (ENUM_INITED != "")
						{
							int pos = ENUM_INITED.length() - ENUM_INITED_POS;
							enums += "\n final int" + ENUM_INITED.substring(0, pos) + "; " + ENUM_INITED.substring(pos + 1);
							ENUM_INITED = "";
						}
						
						enums += "\n}\n";
					}
					break;
				case "entry":
					if (ENUM != "")
					{
						if (is_MAV_CMD)
						{
							if (MAV_CMD == "")
								MAV_CMD +=  ENUM + ";\n final int\n";
						}
						else enums +=  ENUM;
						ENUM = "";
					}
					
					if (is_INITED)
					{
						if (breaked)
						{
							ENUM_INITED_POS = 2;
							ENUM_INITED += "\n" + ("/**\n" + description + "*/\n" + ENTRY);
						}
						else
							if (description == null || description.trim().length() == 0)
							{
								ENUM_INITED_POS = 2;
								ENUM_INITED += "\n" + ENTRY;
							}
							else
							{
								ENUM_INITED_POS = description.length() + 4;
								ENUM_INITED += "\n" + (ENTRY + "//" + description);
							}
					}
					else
						if (breaked)
						{
							ENUM_NOTINITED_POS = 2;
							ENUM_NOTINITED += "\n" + ("/**\n" + description + "*/\n" + ENTRY);
						}
						else
							if (description == null || description.trim().length() == 0)
							{
								ENUM_NOTINITED_POS = 2;
								ENUM_NOTINITED += "\n" + ENTRY;
							}
							else
							{
								ENUM_NOTINITED_POS = description.length() + 4;
								ENUM_NOTINITED += "\n" + (ENTRY + "//" + description);
							}
					
					break;
				case "param":
					is_PARAM = false;
					return;
				case "message":
					packs += "\n}\n";
					is_opt = false;
					break;
				case "field":
					if (MSG != "")
					{
						packs +=  MSG;
						MSG = "";
					}
					
					packs += "\n" + (breaked ? "/**\n" + description + "*/\n" + FIELD : FIELD + "//" + description);
					break;
				case "description":
					if (MSG != "")
					{
						packs += "\n/**\n" + description + "*/\n" + MSG;
						MSG = "";
					}
					
					if (ENUM != "")
					{
						if (is_MAV_CMD)
						{
							if (MAV_CMD == "")
								MAV_CMD += "\n/**\n" + description + "*/\n" + ENUM + ";\n final int\n";
						}
						else enums += "\n/**\n" + description + "*/\n" + ENUM;
						ENUM = "";
					}
					return;
			}
			
			breaked = false;
			description = null;
		}
	}
	
	static String path = "";
	
	static boolean skip = true;
	// command argument: path to the folder with MavLink XML description files
	public static void main(String[] args) {
		
		final MyHandler handler = new MyHandler();
		File            dir     = new File(path = args[0]);
		File[]          files   = dir.listFiles((dir1, name) -> name.endsWith(".xml"));
		
		String protocol = "SimpleProtocol";
		try
		{
			if (files != null)
				for (File file : files)
				{
					SAXParserFactory saxParserFactory = SAXParserFactory.newInstance();
					SAXParser        saxParser        = saxParserFactory.newSAXParser();
					
					msgs_count = 0;
					
					skip = true;
					
					saxParser.parse(file, new DefaultHandler() {
						boolean is_File = false;
						
						@Override public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException {
							if (qName.equals("message")) msgs_count++;
							else
								if (qName.equals("include")) is_File = true;
						}
						
						@Override public void characters(char[] chs, int start, int length) throws SAXException {
							super.characters(chs, start, length);
							for (; (chs[start] < 33) && 0 < length; start++, length--) ;
							if (length < 1) return;
							
							if (is_File)//include other xml descriptor
							{
								skip = false;
								is_File = false;
								try
								{
									
									SAXParserFactory saxParserFactory = SAXParserFactory.newInstance();
									SAXParser        saxParser        = saxParserFactory.newSAXParser();
									
									saxParser.parse(new File(path + File.separator + new String(chs, start, length)), this);
								} catch (Exception e)
								{
									e.printStackTrace();
								}
							}
						}
					});
					
					if (skip) continue;
					msgs_part = msgs_count / 3 + 1;
					
					handler.packs = "";
					handler.enums = "";
					MAV_CMD = "";
					
					handler.parse(file);
					String[] splited = handler.packs.split("@@@@@@@%%%%%@@@@@");//marker  - split point
					
					Files.write(Paths.get(file.getPath().replace(".xml", ".java")),
							("import org.unirail.BlackBox.*;\n" +
							 "public class " + file.getName().replace(".xml", "") + "{}\n " +
							 "class CommunicationChannel extends " + protocol + " implements GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface {}\n" +
							 "class GroundControl implements  InJAVA, InCS{\n"
							 + "interface CommunicationInterface extends GroundControlHandledPacks, CommonPacks {}"
							 + "interface GroundControlHandledPacks  { " + splited[0] + " }\n"
							 + "interface CommonPacks {  " + splited[1] + " }}\n" +
							 "class MicroAirVehicle implements  InC{\n"
							 + "interface CommunicationInterface extends MicroAirVehicleHandledPacks, GroundControl.CommonPacks {}\n"
							 + "interface MicroAirVehicleHandledPacks  {\n" + splited[2] + "}}\n" +
							 (MAV_CMD == "" ? "" : MAV_CMD.substring(0, MAV_CMD.lastIndexOf(",")) + ";\n}\n") +
							 handler.enums).getBytes("UTF8"));
					
					protocol = protocol.equals("SimpleProtocol") ? "AdvancedProtocol" : "SimpleProtocol";
				}
		} catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}

