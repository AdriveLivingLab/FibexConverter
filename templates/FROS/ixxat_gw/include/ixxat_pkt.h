#ifndef IXXAT_ROS_DRIVER_IXXAT_PKT_H_
#define IXXAT_ROS_DRIVER_IXXAT_PKT_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace ixxat_gw_{{channel_name}}
{
	// --------------------------------------
	// Macros
	// --------------------------------------

#define PKT_MSGLEN_OFFSET (4u)
#define PKT_HEADER_LEN (21u) // with SIGN
#define PKT_SIGN_START "B "
#define PKT_SIGN_END "\r\n"
#define ASCII_SPACE (0x10)
#define ASCII_CR (0x0D)
#define ASCII_LF (0x0A)
#define ASCII_B (0x42)

	/***************************************
	 * Typedef
	 ***************************************/

	typedef unsigned char U08;
	typedef unsigned short U16;
	typedef unsigned int U32;
	typedef unsigned long U64;

	typedef unsigned char UINT8;
	typedef unsigned char UINT08;
	typedef unsigned short UINT16;
	typedef unsigned int UINT32;
	typedef unsigned long UINT64;

	/***************************************
	 * Structures
	 ***************************************/

	// IXXAT Packet header structure
	typedef struct __attribute__((__packed__)) pkt
	{
		U16 w_msgSign;		// B space
		U16 w_msglen;		// message complete length (incl. msglen)
		U16 reserved;		// Reserved during wildcard mapping
		U08 iSlotNmbr;		// Slot number of the current frame
		U08 iCycleNmbr;		// Cycle number of the current frame
		U08 b_format;	  	// message frame format information
		U64 qw_timestamp; 	// message timestamp per local gateway time
		U16 w_reserved;	  	// reserved data for future use per GenEthernet standard
		U16 w_datalen;	  	// Number of data bytes (length of ab_data)
		U08 ab_data[];	  	// Data array
	} IXXAT_BIN_PKT;

	// App structure
	typedef struct app_pkt_parse
	{
		char *pSBuf; // Socket Received data buffer
		U32 iSLen;	 // Socket Received data length

		U32 iSIdx; // Start index
		U32 iNIdx; // Next: End + 1 index

		int iRByt;			 // Remaining bytes
		IXXAT_BIN_PKT *pPkt; // Packet buffer pointer
	} APP_IXX_PP;

	/***************************************
	 * Functions
	 ***************************************/

	// Swap 2 bytes
	UINT16 swapOrderU16(UINT16 us)
	{
		us = (us >> 8) |
			 (us << 8);

		return us;
	}

	// Swap 4 bytes
	UINT32 swapOrderU32(UINT32 ui)
	{
		ui = (ui >> 24) |
			 ((ui << 8) & 0x00FF0000) |
			 ((ui >> 8) & 0x0000FF00) |
			 (ui << 24);

		return ui;
	}

	// Swap 8 bytes
	UINT64 swapOrderU64(UINT64 ull)
	{
		ull = (ull >> 56) |
			  ((ull << 40) & 0x00FF000000000000) |
			  ((ull << 24) & 0x0000FF0000000000) |
			  ((ull << 8) & 0x000000FF00000000) |
			  ((ull >> 8) & 0x00000000FF000000) |
			  ((ull >> 24) & 0x0000000000FF0000) |
			  ((ull >> 40) & 0x000000000000FF00) |
			  (ull << 56);

		return ull;
	}

	// Function to get Ixxat Packet from buffer
	int exIxxPkt(APP_IXX_PP *pApp)
	{
		char *pBuf = pApp->pSBuf; // + pApp->iSIdx;
		IXXAT_BIN_PKT *pRawPkt;
		IXXAT_BIN_PKT *pIxPkt;

		int retVal = 0;

		// look for start string "B "
		while (pApp->iSIdx < pApp->iSLen)
		{

			// compare start signature
			if (strcmp((pBuf + pApp->iSIdx), PKT_SIGN_START) != 0)
			{
				pApp->iSIdx++;
				continue;
			}

			// check how many bytes are remaining
			pApp->iRByt = pApp->iSLen - pApp->iSIdx;
			if (pApp->iRByt < PKT_MSGLEN_OFFSET)
			{
				// printf("Partial-1 r: %d, p:%d, l:%d\n", pApp->iRByt, pApp->iSIdx, pApp->iSLen);
				return 1;
			}

			// RAW packet structure mapping
			pRawPkt = (IXXAT_BIN_PKT *)(pBuf + pApp->iSIdx);

			// extract message length
			int pktLen = swapOrderU16(pRawPkt->w_msglen);

			// check if we have enough remaining bytes to read packet length
			if (pApp->iRByt < (pktLen + 2 + 2))
			{
				// printf("Partial-2 r: %d, p:%d, l:%d\n", pApp->iRByt, pApp->iSIdx, pApp->iSLen);
				// std::cout << "RByt: " << pApp->iRByt << " pktLen: " << pktLen << std::endl;
				return 2;
			}

			// buffer for bytes-in-order packet
			pIxPkt = (IXXAT_BIN_PKT *)malloc(pktLen + 2);

			// store the address
			pApp->pPkt = pIxPkt;

			// parse packet
			pIxPkt->w_msgSign = swapOrderU16(pRawPkt->w_msgSign);
			pIxPkt->w_msglen = swapOrderU16(pRawPkt->w_msglen);
			pIxPkt->iCycleNmbr = pRawPkt->iCycleNmbr;
			pIxPkt->iSlotNmbr = pRawPkt->iSlotNmbr;
			pIxPkt->b_format = pRawPkt->b_format;
			pIxPkt->qw_timestamp = swapOrderU64(pRawPkt->qw_timestamp);
			pIxPkt->w_reserved = swapOrderU16(pRawPkt->w_reserved);
			pIxPkt->w_datalen = swapOrderU16(pRawPkt->w_datalen);
			
			// copy the data array
			memcpy(pIxPkt->ab_data, pRawPkt->ab_data, pIxPkt->w_datalen);

			// check if we have the optional space between the data field and the tail + PKT_END signature
			// +1 => 2 for start sign and '- 1' for last index
			pApp->iNIdx = pApp->iSIdx + pIxPkt->w_msglen + 2;

			// TODO: nice to have iNIdx check that it does not exceed packet length
			// Check if optional SPACE is available
			if (pBuf[pApp->iNIdx] != ASCII_CR)
			{
				pApp->iNIdx++;
			}

			// Check end signature
			if (pBuf[pApp->iNIdx] == ASCII_B)
			{
				printf("*Warn. Direct Start sign: %x, %x, %x\n", pBuf[pApp->iNIdx - 1], pBuf[pApp->iNIdx], pBuf[pApp->iNIdx + 1]);
			}
			else if ((pBuf[pApp->iNIdx] != ASCII_CR) || (pBuf[pApp->iNIdx + 1] != ASCII_LF))
			{
				printf("*Warn. End sign: %x, %x, %x\n", pBuf[pApp->iNIdx - 1], pBuf[pApp->iNIdx], pBuf[pApp->iNIdx + 1]);
			}
			else
			{
				pApp->iNIdx += 2;
			}

			// reset remaining bytes
			pApp->iRByt = 0;

			// return packet found
			return 0;
		}

		// return packet not found
		return -1;
	}
} // namespace ixxat_gw_{{channel_name}}
#endif