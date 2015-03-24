#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkSignedDanielssonDistanceMapImageFilter.h"
#include "itkRecursiveGaussianImageFilter.h"
#include "itkGradientImageFilter.h"
#include "itkAverageOutwardFluxImageFilter.h"
#include "itkMedialCurveImageFilter.h"
#include "itkThresholdImageFilter.h"

#include "itkPluginUtilities.h"

#include "CenterlineExtractionCLIModuleCLP.h"

// Use an anonymous namespace to keep class types and function names
// from colliding when module is used as shared object module.  Every
// thing should be in an anonymous namespace except for the module
// entry point, e.g. main()
//
namespace
{

template <class T>
int DoIt( int argc, char * argv[], T )
{
  PARSE_ARGS;

  typedef    T PixelType;
  typedef    T PixelType;

  const unsigned int Dimension = 3;

  typedef itk::Image< PixelType, Dimension >        InputImageType;

  typedef float OutputPixelType;
  typedef itk::Image< OutputPixelType, Dimension >  OutputImageType;

  typedef unsigned short                            VoronoiPixelType;
  typedef itk::Image< VoronoiPixelType, Dimension > VoronoiImageType;

  typedef itk::ImageFileReader< InputImageType >    ReaderType;
  typename ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( inputVolume.c_str() );
  try
  {
    reader->Update();
  }
  catch (itk::ExceptionObject &ex)
  {
    std::cout << ex << std::endl;
    return EXIT_FAILURE;
  }

  // itkSignedDanielssonDistanceMapImageFilter seems to give better results than itkSignedMaurerDistanceMapImageFilter when combined with medial curve extraction
  typedef  itk::SignedDanielssonDistanceMapImageFilter< InputImageType, OutputImageType, VoronoiImageType >  SignedDanielssonDistanceMapImageFilterType;
  typename SignedDanielssonDistanceMapImageFilterType::Pointer distanceMapImageFilter =  SignedDanielssonDistanceMapImageFilterType::New();
  distanceMapImageFilter->SetInput(reader->GetOutput());

  typedef itk::RecursiveGaussianImageFilter< OutputImageType, OutputImageType > RecursiveGaussianFilterType;
  typename RecursiveGaussianFilterType::Pointer gaussianFilterX = RecursiveGaussianFilterType::New();
  typename RecursiveGaussianFilterType::Pointer gaussianFilterY = RecursiveGaussianFilterType::New();
  typename RecursiveGaussianFilterType::Pointer gaussianFilterZ = RecursiveGaussianFilterType::New();

  gaussianFilterX->SetDirection( 0 );
  gaussianFilterY->SetDirection( 1 );
  gaussianFilterZ->SetDirection( 2 );

  gaussianFilterX->SetOrder( RecursiveGaussianFilterType::ZeroOrder );
  gaussianFilterY->SetOrder( RecursiveGaussianFilterType::ZeroOrder );
  gaussianFilterZ->SetOrder( RecursiveGaussianFilterType::ZeroOrder );

  gaussianFilterX->SetNormalizeAcrossScale( false );
  gaussianFilterY->SetNormalizeAcrossScale( false );
  gaussianFilterZ->SetNormalizeAcrossScale( false );

  gaussianFilterX->SetInput( distanceMapImageFilter->GetOutput() );
  gaussianFilterY->SetInput( gaussianFilterX->GetOutput() );
  gaussianFilterZ->SetInput( gaussianFilterY->GetOutput() );

  gaussianFilterX->SetSigma( sigma );
  gaussianFilterY->SetSigma( sigma );
  gaussianFilterZ->SetSigma( sigma );

  typedef itk::ThresholdImageFilter< OutputImageType > ThresholdImageFilterType;
  typename ThresholdImageFilterType::Pointer thresholdFilter = ThresholdImageFilterType::New();

  thresholdFilter->SetInput(gaussianFilterZ->GetOutput());
  thresholdFilter->ThresholdOutside(-100, 100);
  thresholdFilter->SetOutsideValue(100);

  // Compute the gradient.      
  typedef itk::GradientImageFilter< OutputImageType, OutputPixelType, OutputPixelType > GradientFilterType;
  typename GradientFilterType::Pointer gradientFilter = GradientFilterType::New();
  gradientFilter->SetInput( gaussianFilterZ->GetOutput() );
  //gradientFilter->SetInput( gaussianFilterY->GetOutput() );

  // Compute the average outward flux.
  typedef itk::AverageOutwardFluxImageFilter< OutputImageType, OutputPixelType, GradientFilterType::OutputImageType::PixelType > AOFFilterType;
  AOFFilterType::Pointer aofFilter = AOFFilterType::New();
  aofFilter->SetInput( thresholdFilter->GetOutput() );
  aofFilter->SetGradientImage( gradientFilter->GetOutput() );

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 2. Compute the skeleton
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  typedef itk::MedialCurveImageFilter< OutputImageType > MedialCurveFilter;
  MedialCurveFilter::Pointer medialFilter = MedialCurveFilter::New();
  medialFilter->SetInput( thresholdFilter->GetOutput() );
  medialFilter->SetAverageOutwardFluxImage( aofFilter->GetOutput() );
  medialFilter->SetThreshold( threshold );

  // output to file
  typedef itk::ImageFileWriter< MedialCurveFilter::TOutputImage > WriterType;
  typename WriterType::Pointer writer = WriterType::New();
  writer->SetInput( medialFilter->GetOutput() );
  writer->SetFileName( outputVolume.c_str());
  writer->SetUseCompression(1);  

  try
  {
    writer->Update();
  }
  catch (itk::ExceptionObject &ex)
  {
    std::cout << ex << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

} // end of anonymous namespace

int main( int argc, char * argv[] )
{
  PARSE_ARGS;

  itk::ImageIOBase::IOPixelType     pixelType;
  itk::ImageIOBase::IOComponentType componentType;

  try
    {
    itk::GetImageType(inputVolume, pixelType, componentType);

    // This filter handles all types on input, but only produces
    // signed types
    switch( componentType )
      {
      case itk::ImageIOBase::UCHAR:
        return DoIt( argc, argv, static_cast<unsigned char>(0) );
        break;
      case itk::ImageIOBase::CHAR:
        return DoIt( argc, argv, static_cast<char>(0) );
        break;
      case itk::ImageIOBase::USHORT:
        return DoIt( argc, argv, static_cast<unsigned short>(0) );
        break;
      case itk::ImageIOBase::SHORT:
        return DoIt( argc, argv, static_cast<short>(0) );
        break;
      case itk::ImageIOBase::UINT:
        return DoIt( argc, argv, static_cast<unsigned int>(0) );
        break;
      case itk::ImageIOBase::INT:
        return DoIt( argc, argv, static_cast<int>(0) );
        break;
      case itk::ImageIOBase::ULONG:
        return DoIt( argc, argv, static_cast<unsigned long>(0) );
        break;
      case itk::ImageIOBase::LONG:
        return DoIt( argc, argv, static_cast<long>(0) );
        break;
      case itk::ImageIOBase::FLOAT:
        return DoIt( argc, argv, static_cast<float>(0) );
        break;
      case itk::ImageIOBase::DOUBLE:
        return DoIt( argc, argv, static_cast<double>(0) );
        break;
      case itk::ImageIOBase::UNKNOWNCOMPONENTTYPE:
      default:
        std::cout << "unknown component type" << std::endl;
        break;
      }
    }

  catch( itk::ExceptionObject & excep )
    {
    std::cerr << argv[0] << ": exception caught !" << std::endl;
    std::cerr << excep << std::endl;
    return EXIT_FAILURE;
    }
  return EXIT_SUCCESS;
}
