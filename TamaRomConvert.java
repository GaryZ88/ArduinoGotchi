import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class TamaRomConvert {
	private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();
	private static final String lineSeparator = System.getProperty("line.separator");
	
	public static String byteToHex(byte b) {
	    char[] hexChars = new char[2];
        int v = b & 0xFF;
        hexChars[0] = HEX_ARRAY[v >>> 4];
	    hexChars[1] = HEX_ARRAY[v & 0x0F];
	    return new String(hexChars);
	}
		
	public static void main(String argv[]) {
		if (argv.length!=1) {
			System.out.println("Usage: java TamaRomConvert [Tamagotchi P1 ROM File]");
			return;
		}
		String inFile = argv[0];
		System.out.println("Reading Tamagotchi P1 ROM [" + inFile +"]....");
		try {
			Path path = Paths.get(inFile);
			byte[] data = Files.readAllBytes(path);
			System.out.print("ROM Size: " + data.length);
			if (data.length==12288) {
				System.out.println(" - Correct size");
			} else {
				System.out.println(" - Incorrect size! Expected: 12288 bytes");
				return;
			}
			byte[] b = new byte[3];
			BufferedWriter writer=null;
			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("rom_12bit.h", false),"UTF-8"));
			writer.write("static const unsigned char g_program_b12[] PROGMEM = {" + lineSeparator);
			for (int i = 0; i < (data.length/4); i++) {
//				System.out.print(byteToHex(data[i]) + ",");
				if ((i % 6)==0) writer.write("  ");
				int v1 = data[i*4];   // 0F
				int v2 = data[i*4+1]; // A2
				int v3 = data[i*4+2]; // 0C
				int v4 = data[i*4+3]; // 87
				b[0] = (byte)(v1 << 4 | ((v2 >> 4) & 0xF));
				b[1] = (byte)(((v2 & 0xF) << 4) | v3);
				b[2] = (byte)v4;
				writer.write("0x" + byteToHex(b[0]) + ",");
				writer.write("0x" + byteToHex(b[1]) + ",");
				writer.write("0x" + byteToHex(b[2]) + ",");
				writer.write(" ");
				if (((i+1) % 6)==0) {
					writer.write(lineSeparator);
				} 
			}
			writer.write("};" + lineSeparator);
			writer.close();
			System.out.println("[rom_12bit.h] header file generated successfully! Enjoy.");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
