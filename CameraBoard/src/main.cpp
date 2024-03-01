#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>

//Definimos el formato del pixel
struct Pixel {
    uint8_t r, g, b;
    Pixel() = default;
    Pixel(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
};

//Creamos la funcion para convertir la imagen de NV12 a RGB
auto NV12toRGB2(const std::vector<uint8_t> &nv12, int width, int height){
    //La primera parte del archivo define la luminancia (Y)
    int yLimit = width*height;
    auto y = std::vector<uint8_t>();
    y.assign(nv12.begin(), nv12.begin() + yLimit);
    //La segunda parte define la crominancia (Cb y Cr)
    auto cbcr = std::vector<uint8_t>();
    cbcr.assign(nv12.begin() + yLimit, nv12.end());

    auto cb = std::vector<uint8_t>(width*height);
    auto cr = std::vector<uint8_t>(width*height);

    //Definimos la conversion de luminancia y crominancia a RGB
    auto fromNV12 = [](double Y, double Cb, double Cr) {
        return Pixel{
            static_cast<uint8_t>(std::min(255.0, std::max(0.0, Y + 1.370705 * Cr))),
            static_cast<uint8_t>(std::min(255.0, std::max(0.0, Y - 0.337633 * Cb - 0.698001 * Cr))),
            static_cast<uint8_t>(std::min(255.0, std::max(0.0, Y + 1.732446 * Cb)))
        };
    };
    int index = 0;
    auto planar = std::vector<Pixel>(width*height);

    //Recorremos el archivo NV12 aplicando la conversion
    for (int row = 0; row < height; row+=2){
        for (int col = 0; col < width; col+=2){
            double Y1 = y[row * width + col] - 16;
            double Y2 = y[row * width + col + 1] - 16;
            double Y3 = y[(row + 1) * width + col] - 16;
            double Y4 = y[(row + 1) * width + col + 1] - 16;
            double Cb = cbcr[index] - 128 , Cr = cbcr[index + 1] - 128 ;
            planar[row * width + col] = fromNV12(Y1, Cb, Cr);
            planar[row * width + col + 1] = fromNV12(Y2, Cb, Cr);
            planar[(row + 1) * width + col] = fromNV12(Y3, Cb, Cr);
            planar[(row + 1) * width + col + 1] = fromNV12(Y4, Cb, Cr);

            index += 2;
        }
    }
    return planar;
}

std::vector<double> CalculateNDVI(std::vector<Pixel> red, std::vector<Pixel> nir)
{
    auto auxNDVI = std::vector<double>{};
    auxNDVI.reserve(red.size());
    for (int i = 0; i < static_cast<int>(red.size()); ++i) {
        //auto nirValue = (static_cast<double>(nir[i].r + nir[i].g + nir[i].b)/3);
        auto nirValue = static_cast<double>(nir[i].r);
        //nirValue = std::min(std::max(0.0, nirValue), 255.0);
        auto value = (nirValue - red[i].r)/(nirValue + red[i].r);
        auxNDVI.push_back(value);
    }
    return auxNDVI;
}

int main(int argc, char* argv[]) {
    //Definimos las dimensiones de la imagen
    const int width = 4056;
    const int height = 3040;
    //const int RED_INDEX = 0;
    //const int NIR_INDEX = 1;

    //Comprobamos que el numero de argumentos de entrada sea el correcto
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " input_path output_path" << std::endl;
        return 1;
    }

    //El primer argumento debe ser el directorio de donde se extraen las imagenes
    //No tiene por que ser el directorio final, puede ser una carpeta con mas carpetas en su interior
    //El segundo argumento es el directorio en donde se guardan las imagenes
    //El programa creara la carpeta final si esta no existe, las anteriores deben existir
    std::filesystem::path input_path(argv[1]);
    std::filesystem::path output_path(argv[2]);

    if (!std::filesystem::exists(input_path)) {
        std::cerr << "Input image does not exist." << std::endl;
        return 2;
    }

    if (!std::filesystem::exists(output_path.parent_path())) {
        std::filesystem::create_directory(output_path.parent_path());
    }

    std::ifstream input_stream(input_path, std::ios::binary);
    input_stream.seekg(0, std::ios::end);
    auto input_size = input_stream.tellg();
    input_stream.seekg(0, std::ios::beg);

    auto input_data = std::vector<uint8_t>(input_size);

    input_stream.read((char *) input_data.data(), input_size);
    auto rgbPlanar = NV12toRGB2(input_data, width, height);
    /*
    if (argc == 4) {
        std::filesystem::path calibrationPath(argv[3]);
        if (!std::filesystem::exists(calibrationPath)) {
            std::cerr << "Calibration directory does not exist." << std::endl;
            return 3;
        }

        auto whiteCalPath = calibrationPath / "white.ppm";
        auto blackCalPath = calibrationPath / "black.ppm";

        if (!std::filesystem::exists(whiteCalPath) || !std::filesystem::exists(blackCalPath)) {
            std::cerr << "Calibration directory is empty." << std::endl;
            return 4;
        }
        
        auto readPPM = [] (std::filesystem::path &inputPath) {
            auto imageStream = std::ifstream(inputPath, std::ios::binary);
            imageStream.seekg(0, std::ios::end);
            auto fileSize = imageStream.tellg();
            imageStream.seekg(0, std::ios::beg);

            // Removing header
            std::string fileType;
            std::string widthStr;
            std::string heightStr;
            std::string maxValue;

            long int headerBytes = 0;
            int headerParts = 0;
            bool comment = false;
            for (;;) {
                char currentValue;
                imageStream >> currentValue;
                if (!comment) {
                    if (currentValue == '#') { comment = true; }
                    else if (currentValue == ' ') { headerParts++; }
                    else {
                        if (headerParts == 0) fileType += currentValue;
                        else if (headerParts == 1) widthStr += currentValue;
                        else if (headerParts == 2) heightStr += currentValue;
                        else if (headerParts == 3) maxValue += currentValue;
                    }
                    if (headerParts == 3 && currentValue == '\n') break; 
                } else { if (currentValue == '\n') { comment = false; } }
                headerBytes++;
            }
	    const auto dataSize = fileSize - headerBytes;
            auto imageData = std::vector<uint8_t>(dataSize);
            auto imageWidth = std::stoi(widthStr);
            auto imageHeight = std::stoi(heightStr);
            imageStream.read((char *) imageData.data(), imageData.size());
            auto image = std::vector<Pixel>{};
            image.reserve(imageWidth*imageHeight);
            for (int i = 0; i < imageHeight*imageWidth*3; i+=3) {
                image.emplace_back(
                    imageData[i], imageData[i + 1], imageData[i + 2]
                );
            }
            return image;
        };

        auto whiteCal = readPPM(whiteCalPath);
        auto blackCal = readPPM(blackCalPath);

        auto calPixel = [] (uint8_t pixel, uint8_t white, uint8_t black) {
            double calValue = 0;
            if (pixel <= black) {calValue = static_cast<double>(black)/static_cast<double>(white - black);}
            else if(pixel >= white) {calValue = 1.0;} //static_cast<double>(white - black)/static_cast<double>(white - black)
            else if( white <= black ) {calValue = 0;}
            else {calValue = static_cast<double>(pixel - black)/static_cast<double>(white - black);}
            return static_cast<uint8_t>(std::min(std::max(calValue*255, 0.0), 255.0));
        };

        for (int i = 0; i < static_cast<int>(rgbPlanar.size()); ++i) {
            rgbPlanar[i].r = calPixel(rgbPlanar[i].r, whiteCal[i].r, blackCal[i].r );
            rgbPlanar[i].g = calPixel(rgbPlanar[i].g, whiteCal[i].g, blackCal[i].g );
            rgbPlanar[i].b = calPixel(rgbPlanar[i].b, whiteCal[i].b, blackCal[i].b );
        }

    }
    
    auto bands = std::vector<std::vector<Pixel>>{4};
    const int half_width = width / 2;
    const int half_height = height / 2;
    int band_index = 0;
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            if (row < half_height) {
                if (col < half_width) band_index = 0;
                else band_index = 1;
            } else {
                if (col < half_width) band_index = 2;
                else band_index = 3;
            }

            bands[band_index].push_back(rgbPlanar[row * width + col]);
        }
    }
    
    
    auto ndvi = CalculateNDVI(bands[RED_INDEX], bands[NIR_INDEX]);

    std::ofstream output_stream(output_path, std::ios::binary);
    output_stream << "P5\n#AUTOR: BENTAYGA\n" << half_width << " " << half_height << " 255\n";
    for (const auto &pixelValue : ndvi) {
        output_stream << static_cast<uint8_t>(
            std::max(std::min(((pixelValue + 1)/2)*255, 255.0), 0.0));
    }
    output_stream.close();
    */
    std::ofstream output_stream(output_path, std::ios::binary);
    output_stream << "P6\n#AUTOR: BENTAYGA\n" << width << " " << height << " 255\n";
    for (const auto &pixel : rgbPlanar) {
        output_stream << static_cast<uint8_t>(pixel.r);
        output_stream << static_cast<uint8_t>(pixel.g);
        output_stream << static_cast<uint8_t>(pixel.b);
    }
    output_stream.close();
    input_stream.close();

    return 0;
}
