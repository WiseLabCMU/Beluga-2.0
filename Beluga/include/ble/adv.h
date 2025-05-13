/**
 * @file adv.h
 *
 * @brief
 *
 * @date 5/6/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_DTS_ADV_H
#define BELUGA_DTS_ADV_H

/**
 * Initializes the advertising sets for Beluga
 * @return 0 upon success
 * @return negative error code otherwise
 */
int init_advertising(void);

/**
 * Stops connectable and non-connectable advertising.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int stop_advertising(void);

/**
 * Starts connectable and non-connectable advertising. Does nothing if already
 * advertising.
 * @return 0 upon success
 * @return negative error code otherwise
 */
int start_advertising(void);

#endif // BELUGA_DTS_ADV_H
