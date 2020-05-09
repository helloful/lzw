# -*- coding: cp1252 -*-
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

This python script is used to compress and decompress a given input file
using a dictionary compression method.

    ~ ~ ~

    Function compress(strInputFile, strOutputFile, intBitLen=9, intDicLen=255):
        This function needs the filenames of a input text file and of the output
        text file as *.txt. It compresses the text string from the input text
        file and writes it to the output file in a bits.
        The arguments intBitLen and intDicLen set the initial bit length and
        length of the used dictionary. Standard is a bit length of 9 and a
        dictionary length of 255.
        

    Function decompress(strInputFile, strOutputFile, intBitLen=9, intDicLen=255):
        This function needs the filename of a input text file which contains the
        text string compressed in bits (by using compress-Function). It decom-
        presses the bitstring and writes the plaintext to the output file.
        The arguments intBitLen and intDicLen set the initial bit length and
        length of the used dictionary. Standard is a bit length of 9 and a
        dictionary length of 255.
        

    Function initialize_dic(intDicLen=255):
        Initializes the dictionary which is used to compress text strings. It
        needs the length of the dictionary as argument. Argument is optional,
        standard is 255.
        

    Function dic_search(strInput,dic):
        Function used by compress-Function to find an entry in the ditionary and
        return it's position.
        

    Function size_compare(strInputFile,strOutputFile):
        Function to compare the sizes of two files and calculate the size reduc-
        tion due to compression in bytes and percentage.
        

    Function file_compare(strPlaintextFile, strUncompressedFile):
        Takes the names of two text files and compares them char-wise counting
        mismatches. Also compares length of the text strings in both files. Is
        used to quick-check if the compression and decompression worked without
        errors.

    Function full_test(strPlaintextFile):
        Takes a filename as argument and runs a full test of the program using
        this file. The test consists of compressing, decompressing, comparing
        size and text strings char wise.


~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    The easies way to use this programm is to run the full_test:
     1) Put a *.txt-File in the same folder than python script
     2) Run full_test('*.txt')
     3) Answer the questions with y/n to show additional information
     4) The compressed file will be saved in the folder. Filename will be
        *_c.txt.
     5) The uncompressed file will be saved in the folder. Filename will be
        *_uc.txt.


The file_comparison method shows you whether your compression and decom-
pression caused any errors. Size comparison shows how much disk spaced has
been saved due to compression.


~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Encoding Errors ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        
The given instances were using different encodings. For 'compress6v.txt' it is
needed to change the encoding when decompressing the file. This has to be done
in the decompress-Function by changing the write-method at the end.



~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Document dictionaries ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
        
To document a dictionary you have to change the switch bSaveDic at the end of
compress to True. The dictionary will then be saved to the folder as dic_*.txt
Each entry of the dictionary will be separated by blank spaces (8).

~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


This program has been tested on Win10 (Python 3.5.2) and Debian (CIP-Pool @ TU-BS)
(Python 3.x).


Bitstring package is not installed on PCs @ CIP-Pool. To do so use
   pip3 install --user bitstring
   
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

Sorry for my bad english.

~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """

from bitstring import *
import os
import sys
import math
import codecs






""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Compress the given input file and write to output file
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def compress(strInputFile, strOutputFile, intBitLen=9, intDicLen=255):

    strInput = open(strInputFile, 'r').read()
    file = open(strOutputFile, 'wb')

    dic = initialize_dic(intDicLen)
    B = BitStream()

    while (len(strInput) != 0):
        part, dictpos = dic_search(strInput,dic)
        strInput = strInput[len(part):]
        B.append(BitArray(uint=dictpos, length=intBitLen))
        if (len(dic) == (2**intBitLen)):
            B.append(BitArray(uint=(intDicLen+1), length=intBitLen))
            intBitLen += 1
        if(strInput != ""):
            dic.append(part+strInput[0])
    B.tofile(file)
    file.close()
    
    bSaveDic = False
    
    if bSaveDic == True:
        strDictionaryFile = 'dic_' + strInputFile
        file = open(strDictionaryFile, 'wb')
        for i in range(len(dic)):
            file.write(dic[i].encode('utf8'))
            file.write(8*' '.encode('utf8'))
        file.close()
    return 1
    
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """    





""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Decompress the given compressed string, return decompressed string
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def decompress(strInputFile, strOutputFile, intBitLen=9, intDicLen=255):

    B = ConstBitStream(open(strInputFile, 'rb'))
    lstDecompressed = []
    while B.len-B.pos >= intBitLen:
        intSymbol = B.read('uint:' + str(intBitLen))
        if intSymbol == (intDicLen+1):
            intBitLen += 1
        else:
            lstDecompressed.append(intSymbol)

    dic = initialize_dic(intDicLen)

    strResult = ''
    strWord = chr(lstDecompressed.pop(0))
    strResult = strResult + strWord
    for k in lstDecompressed:
        if k < len(dic):
            entry = dic[k]
        elif k == len(dic):
            entry = strWord + strWord[0]
        else:
            raise ValueError('Error in entry no. %s' % k)
        strResult += entry
        
        dic.append(strWord + entry[0])
 
        strWord = entry
        
    file = open(strOutputFile, 'w')
    file.write(strResult)
# alternativ bei Kodierungsfehlern
##    file = open(strOutputFile, 'wb')
##    strEncoding = 'utf8'
##    file.write(strResult.encode(strEncoding))
##    file.close()
    
    return 1

""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """   





""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Initialize Dictionary with characters 0-255
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def initialize_dic(intDicLen=255):
    dic = [chr(i) for i in range(intDicLen+1)]
    dic.append(chr(intDicLen+1))
    return dic
        
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """





""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Search dictionary for search string. If found, return value and position.
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def dic_search(strInput,dic):
    for i in range(len(dic)-1, 0, -1):
        if strInput.find(dic[i], 0) == 0:
            return [dic[i], dic.index(dic[i])]
        
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """





""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Print a size comparison between compressed and uncompressed files.
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def size_compare(strInputFile,strOutputFile):
    if os.path.isfile(strInputFile) and os.path.isfile(strOutputFile):
        intSizeBefore = os.path.getsize(strInputFile)
        intSizeAfter = os.path.getsize(strOutputFile)
        print('\n|', 12*'-', 'Compression report', 12*'-', '|')
        print('|', 'Before compression:', 5*' ', intSizeBefore, 'bytes', (11-len(str(intSizeBefore)))*' ', '|')
        print('|', 'After compression:', 6*' ', intSizeAfter, 'bytes', (11-len(str(intSizeAfter)))*' ', '|')
        print('|', 44*' ', '|')
        print('|', 'Size reduction (rel.)', 3*' ', round(100*(1-(intSizeAfter/intSizeBefore)),2), '%', 10*' ', '|')
        print('|', 'Size reduction (abs.)', 3*' ', intSizeBefore-intSizeAfter, 'bytes', 8*' ', '|')
        print('|', 44*'-', '|')
    else:
        print('File(s) for comparison not found.\n')
        
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """


""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Print a comparison (char-wise) of both files
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def file_compare(strPlaintextFile, strUncompressedFile):
    strPlaintext = open(strPlaintextFile, 'r').read()
    strUncompressed = open(strUncompressedFile, 'r').read()
    intErrors = 0
    for i in range(min(len(strPlaintext),len(strUncompressed))):
        if strPlaintext[i] != strUncompressed[i]: intErrors += 1
    intErrors += (len(strPlaintext) - len(strUncompressed))
    
    print('\n|', 14*'-', 'File comparison', 13*'-', '|')
    print('|', 'Length (plaintext:', 9*' ', len(strPlaintext))
    print('|', 'Length (uncompressed):', 5*' ', len(strUncompressed))
    print('|', 'Matching size:', 13*' ', len(strPlaintext) == len(strUncompressed))   
    print('|', 44*' ', '|')
    print('|', 'No. of mismatches:', 9*' ', intErrors)
    print('|', 44*'-', '|')    
    
        
""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """





""" ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    Runs a full test of the program: Compress, Decompress, File/Size comparison
    and print of plaintext and decompressed text.
~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """
def full_test(strPlaintextFile):
### Set variables
# Length of dictionary
    DIC_LEN = 255

# Length of each symbol
    BIT_LEN = 9

# Length of print output
    PRINT_LEN = 300

# Name for output files
    strCompressedFile = strPlaintextFile[0:len(strPlaintextFile)-4] + '_c' + strPlaintextFile[len(strPlaintextFile)-4:]
    strUncompressedFile = strPlaintextFile[0:len(strPlaintextFile)-4] + '_uc' + strPlaintextFile[len(strPlaintextFile)-4:]


### Run comparison
# Check variables
# Check Bit-Length
    try:
        if 2**BIT_LEN <= DIC_LEN + 1: raise Exception
    except:
        print('Initialize bit length of '+ str(BIT_LEN) + ' bits is too short for dictionary of size ' \
              + str(DIC_LEN) + '. Please adjust variables and re-run the program. For dictionary of size ' \
              + str(DIC_LEN) + 'you should use a bit length of at least ' + str(int(math.log(DIC_LEN + 1, 2))+1)+'.')
        sys.exit(1)

# Check Dic-Length
    try:
        if not (math.log(DIC_LEN + 1,2)%1) == 0: raise Exception
    except:
        print('The chosen length of dictionary can not be used. Please make sure to use a valid dictionary length, i.e. DIC_LEN = 255.')
        sys.exit(1)

# Show plaintext
    if input('Show plaintext? (y/n): ') == 'y':
        print('Plaintext:\n')
        print(44*'-')
        strPlaintext = open(strPlaintextFile, 'r').read()
        print(strPlaintext[:PRINT_LEN] + '...\n(' + str(max(0,(len(strPlaintext)-PRINT_LEN))), 'more characters)')
        print(44*'-')

# Compress and write to file
    if compress(strPlaintextFile, strCompressedFile) == 1:
        print(strPlaintextFile, 'succesfully compressed. The compressed file has been saved as', \
              strCompressedFile, '.\n')

# Decompress and write to file
    if decompress(strCompressedFile, strUncompressedFile) == 1:
        print(strCompressedFile, 'succesfully decompressed. The decompressed text has been saved as', \
              strUncompressedFile, '.')

# Show output
    if input('Show decompressed text? (y/n): ') == 'y':
        strDecompressed = (open(strUncompressedFile, 'r').read())
        print('Decompressed text:\n')
        print(44*'-')
        strDecompressed = open(strUncompressedFile, 'r').read()
        print(strDecompressed[:PRINT_LEN] + '...\n(' + str(max(0,(len(strDecompressed)-PRINT_LEN))), 'more characters)')
        print(44*'-')

# File compare
    if input('Compare plaintext and decompressed file? (y/n): ') == 'y': file_compare(strPlaintextFile, strUncompressedFile)

# Size compare
    if input('Show compression report? (y/n): ') == 'y' : size_compare(strPlaintextFile, strCompressedFile)

    """ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ """





