#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Wire.h>
#include <WString.h>

#include <array>
#include <cstdint>

#define OLED_MAX_LENGTH (UINT32_C(64))
#define OLED_MAX_LENGTH_INDEX (OLED_MAX_LENGTH - 1)

#define GRID_COUNT (UINT32_C(9))
#define MAZE_CELL_COUNT (GRID_COUNT * GRID_COUNT)


#define OUTER_SIZE (UINT32_C(2))
#define INNER_SIZE (UINT32_C(5))
#define CELL_SIZE (INNER_SIZE + OUTER_SIZE)


/**
 * \defgroup Cell_Status_Flag
 * @{
 */
#define WALL_WEST     (0b00000001) //!<< Wall on the West of this cell
#define WALL_EAST     (0b00000010) //!<< Wall on the East of this cell
#define WALL_NORTH    (0b00000100) //!<< Wall on the North of this cell
#define WALL_SOUTH    (0b00001000) //!<< Wall on the South of this cell
#define WALL_ALL      (0b00001111) //!<< Wall on all sides of this cell
#define I_AM_HERE     (0b10000000) //!<< Indicates robot is currently at this cell
/**@}*/

class OLEDMaze {

public:

  OLEDMaze(TwoWire& wire)
    : wire_(&wire), display_(128, 64, &wire, -1) {
  }

  virtual ~OLEDMaze() {
    this->wire_ = nullptr;
  }

  void init(void) {
    display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display_.clearDisplay();
    display_.setFont();
    display_.setTextColor(SSD1306_WHITE);
    display_.setTextSize(1);

    cell_.fill(0x00);
    for (int i = 0; i < GRID_COUNT; i++) {
      this->cellStatusAt(0, i) |= WALL_NORTH;
      this->cellStatusAt(i, 0) |= WALL_WEST;
      this->cellStatusAt(i, GRID_COUNT - 1) |= WALL_EAST;
      this->cellStatusAt(GRID_COUNT - 1, i) |= WALL_SOUTH;
    }
  }

  /**
   * @brief Set text at certain location on the screen. \n
   * A call to `draw()` must be called after this function
   * inorder to display the content
   * @note If the string is longer than the allowed size
   * of the OLED screen, the overflowed bits will be cutted off
   */
  void putText(int16_t x0, int16_t y0, const String& string) {
    size_t strlen = string.length();

    display_.fillRect(x0, y0, 9 * strlen, 10, SSD1306_BLACK);
    display_.setCursor(x0, y0);
    display_.println(string);
  }


  /**
   * @brief Read/Write/Modify the Cell Status at that location
   * A call to `draw()` must be called after this function 
   * inorder to display the content
   * @retval Reference at that location
   * @note The values contains flags that are ORed together
   * Check out the macros `Cell_Status_Flag` on the top of this file
   */
  constexpr uint16_t& cellStatusAt(unsigned row_idx, unsigned col_idx) {
    return this->cell_.at(row_idx * GRID_COUNT + col_idx);
  }

  /**
   * @brief This function use to mark the current cell that the robot
   * is standing. A call to `draw()` must be called after this function 
   * inorder to display the content
   */
  void markRobotCell(unsigned row_idx, unsigned col_idx) {
    this->cellStatusAt(robot_cell_r, robot_cell_c) &= ~I_AM_HERE;
    robot_cell_r = row_idx;
    robot_cell_c = col_idx;
    this->cellStatusAt(row_idx, col_idx) |= I_AM_HERE;
  }


  /**
   * @brief Display content to OLED \n
   * Since this function actually occupies the bus. user should call
   * this function once only after all content has been modified
   */
  void draw(void) {
    display_.fillRect(0, 0, GRID_COUNT * CELL_SIZE, GRID_COUNT * CELL_SIZE + 1, SSD1306_BLACK);

    // always draw borders
    display_.drawLine(2 * CELL_SIZE, 0, (GRID_COUNT - 2) * CELL_SIZE, 0, SSD1306_WHITE);
    display_.drawLine(0, 2 * CELL_SIZE, 0, (GRID_COUNT - 2) * CELL_SIZE, SSD1306_WHITE);
    display_.drawLine(2* CELL_SIZE, GRID_COUNT * CELL_SIZE, (GRID_COUNT - 2) * CELL_SIZE, GRID_COUNT * CELL_SIZE, SSD1306_WHITE);
    display_.drawLine(GRID_COUNT * CELL_SIZE, 2 * CELL_SIZE, GRID_COUNT * CELL_SIZE, (GRID_COUNT - 2) * CELL_SIZE, SSD1306_WHITE);


    // then start drawing the inner walls
    for (unsigned i = 0; i < GRID_COUNT; i++) {
      for (unsigned j = 0; j < GRID_COUNT; j++) {
        auto& cell = this->cell_.at(i * GRID_COUNT + j);
        int px = j * CELL_SIZE;
        int py = i * CELL_SIZE;

        // Draw UP wall
        if (cell & WALL_NORTH && 0 < i) {
          display_.drawLine(px, py, px + CELL_SIZE, py, SSD1306_WHITE);
        } 
        // Draw LEFT wall
        if (cell & WALL_WEST && 0 < j) {
          display_.drawLine(px, py, px, py + CELL_SIZE, SSD1306_WHITE);
        }
        // Draw RIGHT wall
        if (cell & WALL_EAST && j < GRID_COUNT - 1) {
          display_.drawLine(px + CELL_SIZE, py, px + CELL_SIZE, py + CELL_SIZE, SSD1306_WHITE);
        }
        // Draw BOTTOM wall
        if (cell & WALL_SOUTH && i < GRID_COUNT - 1) {
          display_.drawLine(px, py + CELL_SIZE, px + CELL_SIZE, py + CELL_SIZE, SSD1306_WHITE);
        }

        // if the robot is on this cell. also draw it
        if (cell & I_AM_HERE) {
          Serial.printf("Robot at cell: %d, %d\n", i, j);
          display_.fillCircle(px + 4, py + 4, 1, SSD1306_WHITE);
        }
      }
      
    }
    display_.display();
  }

protected:
  unsigned robot_cell_r = 0;
  unsigned robot_cell_c = 0;
  
  std::array<uint16_t, MAZE_CELL_COUNT> cell_;

private:
  TwoWire* wire_;

  Adafruit_SSD1306 display_;

};