#ifndef BIT_STUFFING_H
#define BIT_STUFFING_H
#include "Includes.h"

template <typename type>
void displayNumberBinary(type val, std::ostream& stream) //����� ����� � �������� ����
{
	const unsigned int  byteSize = 8;
	//����� � �����
	size_t len = sizeof(type) * byteSize;
	//����� � �������� ����
	type mask = 1 << (len - 1);

	for (size_t i = 1; i <= len; i++)
	{
		stream<< (val & mask ? '1' : '0');

		val <<= 1;

		if (i % byteSize == 0)	stream<<' ';
	}
}

//�����  � �������� ����
template <typename ForwardIterator>
void displayContainerBinary(ForwardIterator& first, ForwardIterator& last,std::ostream& stream)
{
	for (auto it = first; it != last; ++it)
		displayNumberBinary(*it, stream);
	stream << endl;
}

template <typename ForwardIterator>
void displayContainer(ForwardIterator& first, ForwardIterator& last, std::ostream& stream)
{
	stream.fill('-');
	stream.width(82);
	stream.setf(ios::uppercase);
	for (auto it = first; it != last; ++it)
		stream<<hex<<(unsigned int)(*it) <<" ";
	stream << endl;
	
}

class Conversion
{//������ �� �������� vector<bit> � vector<byte> � �������s
public:
	static const size_t byteSize = 8;

	static void pushByteToBitVector(vector<bit>& vector, std::bitset<byteSize>& bitBuffer)
	{//�������� ���� � ������� ���� � ����� ������� bit
		for (int i = 0; i < byteSize; i++)
			vector.push_back(bitBuffer[i]);
	}

	static byte getByteFromBitVector(const vector<bit>& bitVector, std::bitset<byteSize>& bitBuffer, size_t byteNumber)
	{//������� ����� � ind ������� ������� inVector
		size_t bitVectorSize = bitVector.size();
		size_t j = 0;
		for (int i = byteNumber * 8; j < byteSize && i < bitVectorSize; i++, j++)
			bitBuffer[j] = bitVector[i];
		//�������� ����� �� 8 ���
		for (; j < byteSize; j++)
			bitBuffer[j] = 0;

		return (byte)bitBuffer.to_ulong();
	}

	static size_t reserveBitVectorCapacity(size_t byteVectLength,size_t qLength)
	{
		return ceil((double)((double)byteVectLength / qLength)) + byteVectLength * byteSize;
	}
	static size_t reserveByteVectorCapacity(size_t bitVectLength)
	{//"������" ���� ������ � �������������� ����
		return ceil((double)bitVectLength / byteSize);
	}

public:
	
	static vector<bit> getBitVectorFromByte(byte data, size_t first_bit = 0, size_t last_bit = byteSize)
	{
		bitset<byteSize> mask = data;
		vector<bit> vector;
		vector.reserve(byteSize);

		for (size_t i = first_bit;i < last_bit;i++)
			vector.push_back(mask[i]);

		return vector;
	}

	static vector<bit> convertByteToBitVector(const vector<byte>& byteVector,size_t bitVectorCapacity)
	{//�������������� vector<char>  � vector<bit>
		vector<bit> bitVector;
		//�������������� ����������� ���������� ������� ������� ���,��� ���������� ���-���������
		bitVector.reserve( bitVectorCapacity);

		//����� ��� �������� ����� � ����
		std::bitset<byteSize> bitBuffer;

		for (const byte& el : byteVector)
		{//��� ������� ����� �� ������� �������
		 //������ � ����� � �����
			bitBuffer = el;
			//���������� ����� � ����� vector<bit>
			pushByteToBitVector(bitVector, bitBuffer);
		}

		return bitVector;
	}

	static vector<byte> convertBitToByteVector(const vector<bit>& bitVector)
	{//�������������� vector<bit>  � vector<char>

		vector<byte> byteVector;
		byteVector.resize(reserveByteVectorCapacity(bitVector.size()));

		//����� ��� �������� ������ � ����
		std::bitset<byteSize> bitBuffer;
		
		for (size_t i = 0; i < byteVector.size(); i++)
		{//��� ������� ����� �� ������� �������
			byteVector[i] = getByteFromBitVector(bitVector, bitBuffer, i);
		}

		return byteVector;
	}

	static byte toByte(bit_vect_iter& bitIt, bit_vect_iter& bitEnd, std::bitset<byteSize>& bitBuffer)
	{
		//�������� ������� �����
		bitBuffer.reset();
		bit_vect_iter octetEnd = std::next(bitIt, byteSize);
		bit_vect_iter bitTo = (octetEnd < bitEnd) ? octetEnd : bitEnd;
	
		for (size_t i = 0; bitIt != bitTo; ++bitIt, i++)
			bitBuffer[i] = *bitIt;
		return (byte)bitBuffer.to_ulong();
	}
	static void toByteVector(byte_vect_iter& byteBegin, byte_vect_iter& byteEnd, bit_vect_iter& bitBegin, bit_vect_iter& bitEnd)
	{
		bit_vect_iter bitIt = bitBegin;
		std::bitset<byteSize> bitBuffer;

		for (auto byteIt = byteBegin; ( byteIt != byteEnd) && (bitIt != bitEnd); ++byteIt)
			(*byteIt) = toByte(bitIt, bitEnd, bitBuffer);

	}

};


class BitStuffing
{
public:
	//����������� ���
	enum class InsertingVale : bit { Zero = 0, One = 1 };
	//����������� ���-���������
	enum class ExecutedBy { Sender, Recepient };
private:
	//Frame Delimiter
	byte _FD;
	vector<bit> _frameDelimiter;
	vector<bit> _senderPattern;
	vector<bit> _recepientPattern;
	//��� ��������� ��� ������� ����� ���������� ������������������
	InsertingVale _z;
public:
	
	void init(byte FD,size_t first_bit = 0,size_t last_bit = Conversion::byteSize, InsertingVale z = InsertingVale::One)
	{
		_FD = FD;
		_z = z;

		_frameDelimiter = Conversion::getBitVectorFromByte(_FD);

		// ���, �������� ����������� � ������, �������� ���� ����������
		_senderPattern = Conversion::getBitVectorFromByte(_FD, first_bit, last_bit);
		_recepientPattern = _senderPattern;
		_recepientPattern.push_back((bit)_z);
	}
	size_t senderPatternLength() { return _senderPattern.size(); }
	size_t recepientPatternLength() { return _recepientPattern.size(); }
	byte getFrameDelimiter() { return _FD; }

	BitStuffing(byte FD, size_t first_bit = 0, size_t last_bit = Conversion::byteSize, InsertingVale z = InsertingVale::One)
	{
		init(FD, first_bit, last_bit,z);
	}
	
	BitStuffing() : BitStuffing(0x7E,0,7,InsertingVale::One) {}
	
	size_t bitStuffing(vector<bit>& bitVector, ExecutedBy who,size_t from,size_t to)
	{
		auto fromIt = std::next(bitVector.begin(),from);
		bit_vect_iter toIt;

		bit_vect_iter it;
		//���� ��� �������� ���-��������, �� ������� ������������������ ������� �� 1 ���
		vector<bit> pattern = (who == ExecutedBy::Sender) ? _senderPattern : _recepientPattern;

		size_t patternSize = pattern.size();
		
		size_t nBitsStuffed = 0;

		while (true)
		{
			toIt = std::prev(bitVector.end(),(bitVector.size() -  to));
			it = std::search(fromIt, toIt, pattern.begin() , pattern.end());
			if (it != toIt)
			{
				if (who == ExecutedBy::Sender)
				{
					fromIt = bitVector.insert(std::next(it, patternSize), (bit)_z);
					to++;
				}
				else
				{
					fromIt = bitVector.erase(std::next(it, patternSize - 1));
					to--;
				}
				
				nBitsStuffed++;
			}
			else break;
		}

		return nBitsStuffed;
	}

	
};

#endif //BIT_STUFFING_H
