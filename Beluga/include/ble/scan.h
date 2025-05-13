/**
 * @file scan.h
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_SCAN_H
#define BELUGA_DTS_SCAN_H

/**
 * Starts scanning for other BLE devices.
 * @return 0 upon success.
 * @return negative error code on error.
 * @note This must be called after advertising is started.
 */
int start_active_scanning(void);

/**
 * Stops scanning for other BLE devices.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int stop_scanning(void);

/**
 * Suspends neighbor list updates from BLE.
 */
void suspend_scanning(void);

/**
 * Resumes neighbor list updates from BLE.
 */
void resume_scanning(void);

#endif // BELUGA_DTS_SCAN_H
