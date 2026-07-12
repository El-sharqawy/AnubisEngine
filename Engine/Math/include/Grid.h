#pragma once

#include <fstream>
#include <sstream>

/**
 * @brief A generic grid structure for spatial partitioning or data organization.
 *
 * This template structure is designed to represent a grid that can hold elements of any type T.
 * It can be used for various applications such as spatial partitioning, game world representation,
 * or data organization in a 2D or 3D space.
 *
 * @tparam T The type of elements stored in the grid.
 */
template <typename T>
class CGrid
{
public:
	/**
	 * @brief Move constructor to efficiently transfer resources from another CGrid instance.
	 *
	 * This constructor initializes a new CGrid instance by taking ownership of the resources
	 * from the provided 'other' instance. It sets the member variables of the new instance
	 * to those of 'other' and nullifies 'other's pointer to prevent double deletion.
	 *
	 * @param other The CGrid instance to move from.
	 */
	CGrid(CGrid&& other) noexcept
		: m_iColumns(other.m_iColumns), m_iRows(other.m_iRows),
		m_llGridSize(other.m_llGridSize), m_bIsInitialized(other.m_bIsInitialized),
		m_stName(std::move(other.m_stName)), m_pGridData(other.m_pGridData)
	{
		other.m_pGridData = nullptr; // The "Steal": old object no longer owns the memory
		other.m_bIsInitialized = false;
	}

	/**
	 * @brief Move assignment operator using the copy-and-swap idiom.
	 *
	 * This operator efficiently transfers ownership of resources from another CGrid instance
	 * to this instance. It utilizes the move constructor to create a temporary object and
	 * then swaps the contents with the current object, ensuring exception safety and proper
	 * resource management.
	 *
	 * @param other The CGrid instance to move from.
	 * @return A reference to this CGrid instance after the move operation.
	 */
	CGrid& operator=(CGrid&& other) noexcept
	{
		CGrid temp(std::move(other));	// Use move constructor
		this->Swap(temp);				// Swap our empty state with the new state
		return *this;
	}

	// Disable copy semantics
	CGrid(const CGrid&) = delete; // No copy constructor
	CGrid& operator=(const CGrid&) = delete; // No copy assignment

	/**
	 * @brief Default constructor to create an empty grid.
	 *
	 * This constructor initializes an empty grid with no allocated data.
	 * The grid dimensions and size are set to zero, and the initialized flag is false.
	 */
	CGrid() = default;

	/**
	 * @brief Parameterized constructor to initialize the grid with specified dimensions and initial value.
	 *
	 * This constructor initializes the grid with the given number of columns and rows,
	 * and fills all grid cells with the provided initial value.
	 *
	 * @param iColumns The number of columns in the grid.
	 * @param iRows The number of rows in the grid.
	 * @param initValue The initial value to fill each cell of the grid (default is default-constructed T).
	 */
	explicit CGrid(int32_t iColumns, int32_t iRows, T initValue)	// Use explicit to prevent accidental conversions
	{
		Initialize(iColumns, iRows, initValue);
	}

	/**
	 * @brief Parameterized constructor to initialize the grid with specified dimensions.
	 *
	 * This constructor initializes the grid with the given number of columns and rows.
	 *
	 * @param iColumns The number of columns in the grid.
	 * @param iRows The number of rows in the grid.
	 */
	explicit CGrid(int32_t iColumns, int32_t iRows)	// Use explicit to prevent accidental conversions
	{
		Initialize(iColumns, iRows);
	}

	void Destroy()
	{
		if (m_pGridData)
		{
			std::free(m_pGridData);
			m_pGridData = nullptr;
		}
	}
	/**
	 * @brief Destructor to clean up allocated resources.
	 *
	 * This destructor releases the memory allocated for the grid data array
	 * if it has been initialized.
	 */
	~CGrid()
	{
		if (m_pGridData)
		{
			std::free(m_pGridData);
			m_pGridData = nullptr;
		}
		m_iColumns = 0;
		m_iRows = 0;
		m_llGridSize = 0;
		m_bIsInitialized = false;
	}
	
	/**
	 * @brief Initializes the grid with specified dimensions and initial value.
	 *
	 * This method allocates memory for the grid data array based on the provided
	 * number of columns and rows, and fills all grid cells with the given initial value.
	 *
	 * @param iColumns The number of columns in the grid.
	 * @param iRows The number of rows in the grid.
	 * @param initValue The initial value to fill each cell of the grid (default is default-constructed T).
	 * @return true if initialization was successful; false otherwise.
	 */
	bool Initialize(int32_t iColumns, int32_t iRows, T initValue)
	{
		// Validate input dimensions
		if (iColumns <= 0 || iRows <= 0)
		{
			return (false);
		}

		// Set grid dimensions
		m_iColumns = iColumns;
		m_iRows = iRows;
		m_llGridSize = iColumns * iRows;

		// Clean up existing data if any
		if (m_pGridData)
		{
			std::free(m_pGridData);
		}

		// Allocate memory for the grid data
		m_pGridData = (T*)malloc(m_iColumns * m_iRows * sizeof(T));

		// Initialize all grid cells with the provided initial value
		std::fill(m_pGridData, m_pGridData + m_llGridSize, initValue);

		// Set initialized flag
		SetInitialized(true);
		return (true);
	}

	/**
	 * @brief Initializes the grid with specified dimensions.
	 *
	 * This method allocates memory for the grid data array based on the provided
	 * number of columns and rows.
	 *
	 * @param iColumns The number of columns in the grid.
	 * @param iRows The number of rows in the grid.
	 * @return true if initialization was successful; false otherwise.
	 */
	bool Initialize(int32_t iColumns, int32_t iRows)
	{
		// Validate input dimensions
		if (iColumns <= 0 || iRows <= 0)
		{
			return (false);
		}

		// Set grid dimensions
		m_iColumns = iColumns;
		m_iRows = iRows;
		m_llGridSize = iColumns * iRows;

		// Clean up existing data if any
		if (m_pGridData)
		{
			std::free(m_pGridData);
		}

		// Allocate memory for the grid data
		m_pGridData = (T*)malloc(m_iColumns * m_iRows * sizeof(T));

		// Set initialized flag
		SetInitialized(true);
		return (true);
	}

	/**
	 * @brief Swaps the contents of this grid with another grid.
	 *
	 * This method exchanges the member variables of this grid with those of the
	 * provided 'other' grid instance. It uses std::swap to efficiently swap
	 * all relevant members, ensuring that both grids exchange their data and state.
	 * This is a O(1) operation (constant time).
	 *
	 * @param other The CGrid instance to swap with.
	 */
	void Swap(CGrid& other) noexcept
	{
		// Use std::swap for all members
		std::swap(m_pGridData, other.m_pGridData);
		std::swap(m_iColumns, other.m_iColumns);
		std::swap(m_iRows, other.m_iRows);
		std::swap(m_llGridSize, other.m_llGridSize);
		std::swap(m_bIsInitialized, other.m_bIsInitialized);
		std::swap(m_stName, other.m_stName);
	}

	/**
	 * @brief Retrieves a reference to the grid data array.
	 *
	 * This method returns a pointer to the underlying array that holds
	 * the grid elements. It allows direct access to the grid data for
	 * reading or modification.
	 *
	 * @return Pointer to the grid data array.
	 */
	void SetName(const std::string& name)
	{
		m_stName = name;
	}

	/**
	 * @brief Retrieves the name of the grid.
	 *
	 * This method returns the name assigned to the grid, which can be
	 * used for identification or debugging purposes.
	 *
	 * @return The name of the grid as a constant reference to a string.
	 */
	const std::string& GetName() const
	{
		return m_stName;
	}

	/**
	 * @brief Sets the initialization status of the grid.
	 *
	 * This method allows setting the internal flag that indicates
	 * whether the grid has been successfully initialized.
	 *
	 * @param initialized A boolean value to set the initialization status.
	 */
	void SetInitialized(bool initialized)
	{
		m_bIsInitialized = initialized;
	}

	/**
	 * @brief Checks if the grid has been initialized.
	 *
	 * This method returns a boolean indicating whether the grid has been
	 * successfully initialized with valid dimensions and data.
	 *
	 * @return true if the grid is initialized; false otherwise.
	 */
	bool IsInitialized() const
	{
		return m_bIsInitialized;
	}

	/**
	 * @brief Retrieves the number of columns in the grid.
	 *
	 * This method returns the total number of columns that the grid contains.
	 *
	 * @return The number of columns in the grid.
	 */
	int32_t GetColumns() const
	{
		return m_iColumns;
	}

	/**
	 * @brief Retrieves the number of rows in the grid.
	 *
	 * This method returns the total number of rows that the grid contains.
	 *
	 * @return The number of rows in the grid.
	 */
	int32_t GetRows() const
	{
		return m_iRows;
	}

	/**
	 * @brief Retrieves the total size of the grid.
	 *
	 * This method returns the total number of elements in the grid,
	 * calculated as the product of the number of columns and rows.
	 *
	 * @return The total size of the grid.
	 */
	size_t size() const
	{
		return m_llGridSize;
	}

	/**
	 * @brief Retrieves the width of the grid.
	 *
	 * This method returns the number of columns in the grid,
	 * representing its width.
	 *
	 * @return The width of the grid.
	 */
	int32_t GetWidth() const
	{
		return m_iColumns;
	}

	/**
	 * @brief Retrieves the height of the grid.
	 *
	 * This method returns the number of rows in the grid,
	 * representing its height.
	 *
	 * @return The height of the grid.
	 */
	int32_t GetHeight() const
	{
		return m_iRows;
	}

	/**
	 * @brief Subscript operator to access grid elements.
	 *
	 * This operator allows access to the grid elements at the specified index.
	 * It returns a pointer to the element located at the given index in the
	 * grid data array.
	 *
	 * @param Index The index of the grid element to access.
	 * @return Pointer to the grid element at the specified index.
	 */
	T& operator[](int32_t Index)
	{
		static T s_DefaultValue{}; // or make it `const T` and return const&

		if (Index < 0 || Index >= m_llGridSize || m_pGridData == nullptr)
		{
			fprintf(stderr, "CGrid::operator[] - Index out of bounds: %d", Index);
			return { s_DefaultValue };
		}

		return m_pGridData[Index];
	}

	/**
	 * @brief Const subscript operator to access grid elements.
	 *
	 * This operator allows read-only access to the grid elements
	 * at the specified index. It returns a pointer to the element
	 * located at the given index in the grid data array.
	 *
	 * @param Index The index of the grid element to access.
	 * @return Pointer to the grid element at the specified index.
	 */
	const T& operator[](int32_t Index) const
	{
		static T s_DefaultValue{}; // or make it `const T` and return const&

		if (Index < 0 || Index >= m_llGridSize || m_pGridData == nullptr)
		{
			fprintf(stderr, "CGrid::operator[] - Index out of bounds: %d", Index);
			return { s_DefaultValue };
		}

		return m_pGridData[Index];
	}

	/**
	 * @brief Conversion operator to retrieve the grid data array.
	 *
	 * This operator allows implicit conversion of the CGrid instance
	 * to a pointer of type T*, providing direct access to the underlying
	 * grid data array.
	 *
	 * @return Pointer to the grid data array.
	 */
	operator T* ()
	{
		return m_pGridData;
	}

	/**
	 * @brief Const conversion operator to retrieve the grid data array.
	 *
	 * This operator allows implicit conversion of the CGrid instance
	 * to a constant pointer of type T*, providing read-only access to
	 * the underlying grid data array.
	 *
	 * @return Constant pointer to the grid data array.
	 */
	operator const T* () const
	{
		return m_pGridData;
	}

	/**
	 * @brief Method to retrieve the grid data array.
	 *
	 * This method returns a pointer to the underlying grid data array,
	 * allowing direct access to the grid elements for reading or modification.
	 *
	 * @return Pointer to the grid data array.
	 */
	T* data()
	{
		return m_pGridData;
	}

	/**
	 * @brief Const method to retrieve the grid data array.
	 *
	 * This method returns a constant pointer to the underlying grid data array,
	 * allowing read-only access to the grid elements.
	 *
	 * @return Constant pointer to the grid data array.
	 */
	const T* data() const
	{
		return m_pGridData;
	
	}

	/**
	 * @brief Calculates the total size of the grid data in bytes.
	 *
	 * This method computes the total memory size occupied by the grid data array
	 * by multiplying the number of elements in the grid by the size of each element type T.
	 *
	 * @return The total size of the grid data in bytes.
	 */
	size_t GetDataSizeInBytes() const
	{
		return (m_llGridSize * sizeof(T));
	}

	/**
	 * @brief Calculates the linear index for a given column and row.
	 *
	 * This method computes the linear index in the grid data array
	 * based on the provided column and row indices. It returns -1 if
	 * the provided indices are out of bounds.
	 *
	 * @param Col The column index.
	 * @param Row The row index.
	 * @return The linear index corresponding to the (Col, Row) position, or -1 if out of bounds.
	 */
	size_t CalculateIndex(int32_t Col, int32_t Row) const
	{
		// 1. Boundary check: ensure we don't access outside the allocated 131x131
#if defined(_DEBUG)
		if (Col < 0 || Col >= m_iColumns || Row < 0 || Row >= m_iRows) 
		{
			throw std::out_of_range("Invalid column or row index");
		}
#endif

		size_t Index = static_cast<size_t>(Row * m_iColumns + Col);
		return (Index);
	}

	/**
	 * @brief Retrieves a pointer to the grid element at the specified column and row.
	 *
	 * This method calculates the linear index for the given column and row,
	 * and returns a pointer to the corresponding grid element. If the indices
	 * are out of bounds, it returns nullptr.
	 *
	 * @param Col The column index.
	 * @param Row The row index.
	 * @return Pointer to the grid element at (Col, Row), or nullptr if out of bounds.
	 */
	T* GetAddr(int32_t Col, int32_t Row)
	{
		if (Col < 0 || Col >= m_iColumns || Row < 0 || Row >= m_iRows)
		{
			return (GetAddr(0, 0));
		}

		size_t Index = CalculateIndex(Col, Row);
		return &(m_pGridData[Index]);
	}

	/**
	 * @brief Accesses the grid element at the specified column and row with bounds checking.
	 *
	 * This method calculates the linear index for the given column and row,
	 * and returns a reference to the corresponding grid element. If the indices
	 * are out of bounds, it throws an std::out_of_range exception.
	 *
	 * @param Col The column index.
	 * @param Row The row index.
	 * @return Reference to the grid element at (Col, Row).
	 * @throws std::out_of_range if the column or row indices are out of bounds.
	 */
	T& At(int32_t Col, int32_t Row)
	{
		size_t index = CalculateIndex(Col, Row);
		if (index == -1)
		{
			throw std::out_of_range("CGrid::at - Column or Row out of bounds");
		}
		return m_pGridData[index];
	}

	/**
	 * @brief Accesses the grid element at the specified linear index with bounds checking.
	 *
	 * This method returns a reference to the grid element at the given linear index.
	 * If the index is out of bounds, it throws an std::out_of_range exception.
	 *
	 * @param Index The linear index of the grid element to access.
	 * @return Reference to the grid element at the specified index.
	 * @throws std::out_of_range if the index is out of bounds.
	 */
	T& At(int32_t Index)
	{
		if (Index >= m_llGridSize || Index < 0)
		{
			throw std::out_of_range("CGrid::at - Indexw out of bounds");
		}
		return m_pGridData[Index];
	}

	/**
	 * @brief Sets the value of the grid element at the specified column and row.
	 *
	 * This method calculates the linear index for the given column and row,
	 * and sets the corresponding grid element to the provided value. If the
	 * indices are out of bounds, it throws an std::out_of_range exception.
	 *
	 * @param Col The column index.
	 * @param Row The row index.
	 * @param value The value to set at the specified grid position.
	 * @throws std::out_of_range if the column or row indices are out of bounds.
	 */
	void Set(int32_t Col, int32_t Row, const T& value)
	{
		size_t index = CalculateIndex(Col, Row);
		if (index == -1)
		{
			throw std::out_of_range("CGrid::set - Column or Row out of bounds");
		}
		m_pGridData[index] = value;
	}

	/**
	 * @brief Sets the value of the grid element at the specified linear index.
	 *
	 * This method sets the grid element at the given linear index to the
	 * provided value. If the index is out of bounds, it throws an
	 * std::out_of_range exception.
	 *
	 * @param Index The linear index of the grid element to set.
	 * @param value The value to set at the specified index.
	 * @throws std::out_of_range if the index is out of bounds.
	 */
	void Set(int32_t Index, const T& value)
	{
		if (Index >= m_llGridSize || Index < 0)
		{
			throw std::out_of_range("CGrid::set - Index out of bounds");
		}
		m_pGridData[Index] = value;
	}

	/**
	 * @brief Accesses the grid element at the specified column and row with bounds checking.
	 *
	 * This method calculates the linear index for the given column and row,
	 * and returns a reference to the corresponding grid element. If the indices
	 * are out of bounds, it throws an std::out_of_range exception.
	 *
	 * @param Col The column index.
	 * @param Row The row index.
	 * @return Reference to the grid element at (Col, Row).
	 * @throws std::out_of_range if the column or row indices are out of bounds.
	 */
	T& Get(int32_t Col, int32_t Row)
	{
		size_t index = CalculateIndex(Col, Row);
		return m_pGridData[index];
	}

	/**
	 * @brief Accesses the grid element at the specified linear index with bounds checking.
	 *
	 * This method returns a reference to the grid element at the given linear index.
	 * If the index is out of bounds, it throws an std::out_of_range exception.
	 *
	 * @param Index The linear index of the grid element to access.
	 * @return Reference to the grid element at the specified index.
	 * @throws std::out_of_range if the index is out of bounds.
	 */
	const T& Get(int32_t Index) const
	{
		if (Index >= m_llGridSize || Index < 0)
		{
			throw std::out_of_range("CGrid::Get - Index out of bounds");
		}
		return m_pGridData[Index];
	}

	bool SaveToFile(const std::string& fileName) const
	{
		// Validation checks
		if (!IsInitialized())
		{
			fprintf(stderr, "CGrid::SaveToFile - Grid is not initialized.\n");
			return (false);
		}

		// Ensure grid data is valid
		if (!m_pGridData)
		{
			fprintf(stderr, "CGrid::SaveToFile - Grid data is null.\n");
			return (false);
		}

		// Open the file in binary mode
		std::ofstream sFile(fileName, std::ios::binary | std::ios::trunc);

		// Check if the file opened successfully
		if (!sFile.is_open())
		{
			fprintf(stderr, "CGrid::SaveToFile - Failed to open file: %s\n", fileName.c_str());
			return (false);
		}

		sFile.exceptions(std::ofstream::failbit | std::ofstream::badbit); // <-- this is critical

		// 1. Write a "Magic Number" to identify the file type (Optional but professional)
		const uint32_t MAGIC_NUMBER = 0x47726964; // Grid in Hex
		sFile.write(reinterpret_cast<const char*>(&MAGIC_NUMBER), sizeof(MAGIC_NUMBER));

		// 2. Write metadata (Columns and Rows)
		sFile.write(reinterpret_cast<const char*>(&m_iColumns), sizeof(m_iColumns));
		sFile.write(reinterpret_cast<const char*>(&m_iRows), sizeof(m_iRows));

		// 3. Write the raw data block
		sFile.write(reinterpret_cast<const char*>(m_pGridData), GetDataSizeInBytes());

		// 4. Close the file
		sFile.close();

		return (true);
	}

	bool LoadFromFile(const std::string& fileName)
	{
		// Open the file in binary mode
		std::ifstream sFile(fileName, std::ios::binary);

		// Check if the file opened successfully
		if (!sFile.is_open())
		{
			fprintf(stderr, "CGrid::SaveToFile - Failed to open file: %s\n", fileName.c_str());
			return (false);
		}

		// 1. Check Magic Number
		uint32_t magicNumber = 0;
		sFile.read(reinterpret_cast<char*>(&magicNumber), sizeof(magicNumber));
		const uint32_t EXPECTED_MAGIC_NUMBER = 0x47726964; // Grid in Hex
		if (magicNumber != EXPECTED_MAGIC_NUMBER)
		{
			fprintf(stderr, "CGrid::LoadFromFile - Invalid file format: %s\n", fileName.c_str());
			sFile.close();
			return (false);
		}

		// 2. Read metadata (Columns and Rows)
		int32_t iColumns = 0;
		int32_t iRows = 0;
		sFile.read(reinterpret_cast<char*>(&iColumns), sizeof(iColumns));
		sFile.read(reinterpret_cast<char*>(&iRows), sizeof(iRows));

		// 3. Re-initialize the grid to match the file's dimensions
		if (!Initialize(iColumns, iRows))
		{
			fprintf(stderr, "CGrid::LoadFromFile - Failed to initialize grid.\n");
			sFile.close();
			return (false);
		}

		// 4. Read the raw data block
		sFile.read(reinterpret_cast<char*>(m_pGridData), GetDataSizeInBytes());

		// 5. Close the file
		sFile.close();
		return (true);
	}

private:
	// 32 Bytes
	std::string m_stName;								/**< Name of the grid */

	// Usually 8 bytes
	T* m_pGridData = nullptr;							/**< Pointer to the grid data array */

	// 8 bytes (Total element count)
	size_t m_llGridSize = 0;							/**< Size of each grid cell */

	// Each is 4 bytes
	int32_t m_iColumns = 0;								/**< Number of columns in the grid */
	int32_t m_iRows = 0;									/**< Number of rows in the grid */

	// 1 byte
	bool m_bIsInitialized = false;						/**< Flag indicating if the grid is initialized */
	// [3 padding bytes here to align to 8-byte boundary]
};